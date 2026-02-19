//! Setup the IMU to output accels and print them

use std::{sync::Arc, time::Duration};

use clap::Parser;

use microstrain_inertial::{
    api::{
        commands::imu_3dm::DescriptorRate,
        data::{
            Quatf, Vector3f,
            filter::{self, FILTER_DESCRIPTOR_SET, FilterField, FilterPacket},
            sensor,
        },
    },
    interface::AsyncInterface,
    serialport::start_serialport_thread,
};

#[derive(Parser)]
struct Args {
    port: String,
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    env_logger::init();
    let args = Args::parse();

    let interface = Arc::new(AsyncInterface::new());

    // Create background serial port IO which will move data between the interface and serial port
    start_serialport_thread(&args.port, 115200, interface.clone())
        .expect("Failed to launch IO threads");

    const DECIMATION: u16 = 10;
    tokio::time::timeout(
        Duration::from_millis(500),
        interface.set_message_format(
            filter::FILTER_DESCRIPTOR_SET,
            &[
                DescriptorRate {
                    descriptor: filter::AttitudeQuaternion::DESCRIPTOR,
                    decimation: DECIMATION,
                },
                DescriptorRate {
                    descriptor: filter::LinearAccel::DESCRIPTOR,
                    decimation: DECIMATION,
                },
            ],
        ),
    )
    .await
    .expect("Timedout waiting for set_message_format response")
    .expect("Error setting rates");

    // Turn off any sensor messages
    tokio::time::timeout(
        Duration::from_millis(500),
        interface.set_message_format(sensor::SENSOR_DESCRIPTOR_SET, &[]),
    )
    .await
    .expect("Timedout waiting for set_message_format response")
    .expect("Error setting rates");

    fn vector3_str(v: Vector3f) -> String {
        format!("x: {:.3}, y: {:3.3}, z: {:3.3}", v.x, v.y, v.z)
    }

    fn quat_str(q: Quatf) -> String {
        format!("[{:.3}, {:.3}, {:.3}, {:.3}]", q.w, q.x, q.y, q.z)
    }
    loop {
        let data = interface.read_raw_data().await.unwrap();
        if data.descriptor_set() != FILTER_DESCRIPTOR_SET {
            continue;
        }
        let packet = FilterPacket::new(data.payload());

        let header = " ".repeat(14) + "Quat" + &" ".repeat(28) + "Accel";
        println!("{header}");

        let mut data_s = String::new();
        for field in packet.fields() {
            match field {
                Ok(field) => match field {
                    FilterField::AttitudeQuaternion(quat) => {
                        data_s += &format!("{} | ", quat_str(quat.q));
                    }
                    FilterField::LinearAccel(accel) => {
                        data_s += &format!("{}", vector3_str(accel.accel_m_s2));
                    }
                    _ => (),
                },
                Err(e) => {
                    log::error!("Error parsing sensor field: {e:?}");
                }
            }
        }
        println!("{data_s}");
    }
}
