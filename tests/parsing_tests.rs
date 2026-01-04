use microstrain_inertial::{
    api::{
        DataPacket, Packet,
        data::{
            Quatf, Vector3f,
            filter::{
                AttitudeQuaternion, EulerAngles, FILTER_DESCRIPTOR_SET, FilterField, GyroBias,
                ValidFlags,
            },
            sensor::{CompQuaternion, SENSOR_DESCRIPTOR_SET, ScaledAccel, SensorField},
            shared::{DeltaTime, GpsTimestamp, GpsTimestampValidFlags, SharedField},
        },
    },
    parser::MessageParser,
    serialize::OwnedMessage,
};

const SYNC: [u8; 2] = [0x75, 0x65];

fn make_filter_quat_field(quat: [f32; 4]) -> Vec<u8> {
    const LEN: u8 = 20;
    const DESCRIPTOR: u8 = 0x03;
    let valid = 1u16;
    vec![
        vec![LEN, DESCRIPTOR],
        quat[0].to_be_bytes().to_vec(),
        quat[1].to_be_bytes().to_vec(),
        quat[2].to_be_bytes().to_vec(),
        quat[3].to_be_bytes().to_vec(),
        valid.to_be_bytes().to_vec(),
    ]
    .into_iter()
    .flatten()
    .collect()
}

fn make_filter_gyro_bias_field(bias: [f32; 3]) -> Vec<u8> {
    const LEN: u8 = 16;
    const DESCRIPTOR: u8 = 0x06;
    let valid = 1u16;
    vec![
        vec![LEN, DESCRIPTOR],
        bias[0].to_be_bytes().to_vec(),
        bias[1].to_be_bytes().to_vec(),
        bias[2].to_be_bytes().to_vec(),
        valid.to_be_bytes().to_vec(),
    ]
    .into_iter()
    .flatten()
    .collect()
}

fn make_filter_euler_field(eulers: [f32; 3]) -> Vec<u8> {
    const LEN: u8 = 16;
    const DESCRIPTOR: u8 = 0x05;
    let valid = 1u16;
    vec![
        vec![LEN, DESCRIPTOR],
        eulers[0].to_be_bytes().to_vec(),
        eulers[1].to_be_bytes().to_vec(),
        eulers[2].to_be_bytes().to_vec(),
        valid.to_be_bytes().to_vec(),
    ]
    .into_iter()
    .flatten()
    .collect()
}

fn make_sensor_scaled_accel_field(accel: [f32; 3]) -> Vec<u8> {
    const LEN: u8 = 14;
    const DESCRIPTOR: u8 = 0x04;
    vec![
        vec![LEN, DESCRIPTOR],
        accel[0].to_be_bytes().to_vec(),
        accel[1].to_be_bytes().to_vec(),
        accel[2].to_be_bytes().to_vec(),
    ]
    .into_iter()
    .flatten()
    .collect()
}

fn make_sensor_complementary_quat(quat: [f32; 4]) -> Vec<u8> {
    const LEN: u8 = 18;
    const DESCRIPTOR: u8 = 0xA;
    vec![
        vec![LEN, DESCRIPTOR],
        quat[0].to_be_bytes().to_vec(),
        quat[1].to_be_bytes().to_vec(),
        quat[2].to_be_bytes().to_vec(),
        quat[3].to_be_bytes().to_vec(),
    ]
    .into_iter()
    .flatten()
    .collect()
}

fn make_shared_delta_time(seconds: f64) -> Vec<u8> {
    const LEN: u8 = 10;
    const DESCRIPTOR: u8 = 0xD4;
    vec![vec![LEN, DESCRIPTOR], seconds.to_be_bytes().to_vec()]
        .into_iter()
        .flatten()
        .collect()
}

fn make_shared_gps_timestamp(tow: f64, week: u16, valid_flags: u16) -> Vec<u8> {
    const LEN: u8 = 14;
    const DESCRIPTOR: u8 = 0xD3;
    vec![
        vec![LEN, DESCRIPTOR],
        tow.to_be_bytes().to_vec(),
        week.to_be_bytes().to_vec(),
        valid_flags.to_be_bytes().to_vec(),
    ]
    .into_iter()
    .flatten()
    .collect()
}

fn frame_payload(descriptor_set: u8, payload: &[u8]) -> Vec<u8> {
    assert!(payload.len() <= 255);
    let mut frame = SYNC.to_vec();
    frame.push(descriptor_set);
    frame.push(payload.len() as u8);
    frame.extend_from_slice(payload);
    let crc = fletcher::calc_fletcher16(&frame);
    frame.extend_from_slice(&crc.to_be_bytes());
    frame
}

fn parse_bytes(parser: &mut MessageParser, data: &[u8]) -> Vec<OwnedMessage> {
    let mut frames = Vec::new();
    for b in data {
        match parser.push_byte(*b).unwrap() {
            Some(frame) => frames.push(frame.to_owned()),
            None => (),
        }
    }

    frames
}

#[test]
fn test_parsing_multiple_sensor_fields() {
    const QUAT_VALUE: [f32; 4] = [0.1, 0.2, 0.3, 0.4];
    const ACCEL_VALUE: [f32; 3] = [-1.0, -1.1, -1.2];
    const TOW_VALUE: f64 = 5000.1;
    const WEEK_VALUE: u16 = 2300;
    let payload: Vec<u8> = vec![
        make_sensor_complementary_quat(QUAT_VALUE),
        make_sensor_scaled_accel_field(ACCEL_VALUE),
        make_shared_gps_timestamp(TOW_VALUE, WEEK_VALUE, 1),
    ]
    .into_iter()
    .flatten()
    .collect();

    let frame = frame_payload(SENSOR_DESCRIPTOR_SET, &payload);
    let mut parser = MessageParser::new();
    let parsed = parse_bytes(&mut parser, &frame);
    assert_eq!(1, parsed.len());

    let sensor_packet =
        match Packet::from_frame(parsed[0].descriptor_set(), parsed[0].payload()).unwrap() {
            Packet::Command(_) => panic!("Command packet!"),
            Packet::Data(data_packet) => match data_packet {
                DataPacket::SensorPacket(p) => p,
                _ => panic!("wrong data packet"),
            },
        };

    let fields: Result<Vec<SensorField>, _> = sensor_packet.fields().collect();
    let fields = fields.unwrap();
    assert_eq!(3, fields.len());
    assert_eq!(
        SensorField::CompQuaternion(CompQuaternion {
            q: Quatf::from_array(QUAT_VALUE)
        }),
        fields[0]
    );
    assert_eq!(
        SensorField::ScaledAccel(ScaledAccel {
            scaled_accel: Vector3f::from_array(ACCEL_VALUE),
        }),
        fields[1]
    );
    assert_eq!(
        SensorField::Shared(SharedField::GpsTimestamp(GpsTimestamp {
            tow_s: TOW_VALUE,
            week_number: WEEK_VALUE,
            valid_flags: GpsTimestampValidFlags(1)
        })),
        fields[2]
    );
}

#[test]
fn test_parsing_multiple_filter_fields() {
    const QUAT_VALUE: [f32; 4] = [1.2, 3.0, 4.0, 5.0];
    const GYRO_BIAS_VALUE: [f32; 3] = [10.0, 10.1, 10.2];
    const EULER_VALUES: [f32; 3] = [-0.2, -0.3, 1.5];
    const TIME_VALUE: f64 = 12.1234;
    let payload: Vec<u8> = vec![
        make_filter_quat_field(QUAT_VALUE),
        make_filter_gyro_bias_field(GYRO_BIAS_VALUE),
        make_filter_euler_field(EULER_VALUES),
        make_shared_delta_time(TIME_VALUE),
    ]
    .into_iter()
    .flatten()
    .collect();

    let frame = frame_payload(FILTER_DESCRIPTOR_SET, &payload);

    let mut parser = MessageParser::new();
    let parsed = parse_bytes(&mut parser, &frame);
    assert_eq!(1, parsed.len());

    let filter_packet =
        match Packet::from_frame(parsed[0].descriptor_set(), parsed[0].payload()).unwrap() {
            Packet::Command(_) => panic!("Command packet!"),
            Packet::Data(data_packet) => match data_packet {
                DataPacket::FilterPacket(p) => p,
                _ => panic!("wrong data packet"),
            },
        };

    let fields: Result<Vec<FilterField>, _> = filter_packet.fields().collect();
    let fields = fields.unwrap();
    assert_eq!(4, fields.len());
    assert_eq!(
        FilterField::AttitudeQuaternion(AttitudeQuaternion {
            q: Quatf::from_array(QUAT_VALUE),
            valid_flags: ValidFlags(1),
        }),
        fields[0]
    );
    assert_eq!(
        FilterField::GyroBias(GyroBias {
            bias_rad_s: Vector3f::from_array(GYRO_BIAS_VALUE),
            valid_flags: ValidFlags(1)
        }),
        fields[1]
    );
    assert_eq!(
        FilterField::EulerAngles(EulerAngles {
            roll_rad: EULER_VALUES[0],
            pitch_rad: EULER_VALUES[1],
            yaw_rad: EULER_VALUES[2],
            valid_flags: ValidFlags(1)
        }),
        fields[2]
    );
    assert_eq!(
        FilterField::Shared(SharedField::DeltaTime(DeltaTime { dt_s: TIME_VALUE })),
        fields[3]
    );
}
