use microstrain_inertial::{
    api::{
        DataPacket, Packet,
        data::{
            Quatf, Vector3f,
            filter::{AttitudeQuaternion, FilterField, GyroBias, ValidFlags},
            sensor::SensorField,
        },
    },
    parser::MessageParser,
    serialize::OwnedMessage,
};

const SYNC: [u8; 2] = [0x75, 0x65];

fn make_filter_quat_field(quat: [f32; 4], valid: bool) -> Vec<u8> {
    const LEN: u8 = 18;
    const DESCRIPTOR: u8 = 0x03;
    let valid = if valid { 1u16 } else { 0u16 };
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

fn make_filter_gyro_bias(bias: [f32; 3], valid: bool) -> Vec<u8> {
    const LEN: u8 = 14;
    const DESCRIPTOR: u8 = 0x06;
    let valid = if valid { 1u16 } else { 0u16 };
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
    const QUAT_VALUE: [f32; 4] = [1.2, 3.0, 4.0, 5.0];
    const GYRO_BIAS_VALUE: [f32; 3] = [10.0, 10.1, 10.2];
    let payload: Vec<u8> = vec![
        make_filter_quat_field(QUAT_VALUE, true),
        make_filter_gyro_bias(GYRO_BIAS_VALUE, true),
    ]
    .into_iter()
    .flatten()
    .collect();

    let frame = frame_payload(0x82, &payload);

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

    let fields: Vec<FilterField> = filter_packet.fields().flatten().collect();
    assert_eq!(2, fields.len());
    if let FilterField::AttitudeQuaternion(field) = fields[0] {
        assert_eq!(
            AttitudeQuaternion {
                q: Quatf::from_array(QUAT_VALUE),
                valid_flags: ValidFlags(1),
            },
            field
        );
    } else {
        panic!("Unexpected type on field[0]")
    }
    if let FilterField::GyroBias(field) = fields[1] {
        assert_eq!(
            GyroBias {
                bias_rad_s: Vector3f::from_array(GYRO_BIAS_VALUE),
                valid_flags: ValidFlags(1)
            },
            field
        )
    } else {
        panic!("Unexpected type on field[1]")
    }
}
