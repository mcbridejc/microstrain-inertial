//! Setup the IMU to output accels and print them

use std::{sync::Arc, time::Duration};

use clap::Parser;

use microstrain_inertial::{
    api::{
        commands::imu_3dm::DescriptorRate,
        data::sensor::{self, SensorField, SensorPacket},
        types::Vector3f,
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

    const DECIMATION: u16 = 500;
    tokio::time::timeout(
        Duration::from_millis(500),
        interface.set_message_format(
            sensor::SENSOR_DESCRIPTOR_SET,
            &[
                DescriptorRate {
                    descriptor: sensor::ScaledAccel::DESCRIPTOR,
                    decimation: DECIMATION,
                },
                DescriptorRate {
                    descriptor: sensor::ScaledGyro::DESCRIPTOR,
                    decimation: DECIMATION,
                },
                DescriptorRate {
                    descriptor: sensor::ScaledMag::DESCRIPTOR,
                    decimation: DECIMATION,
                },
            ],
        ),
    )
    .await
    .expect("Timedout waiting for set_message_format response")
    .expect("Error setting rates");

    fn vector3_str(v: Vector3f) -> String {
        format!("x: {:.3}, y: {:3.3}, z: {:3.3}", v.x, v.y, v.z)
    }
    loop {
        let data = interface.read_raw_data().await.unwrap();
        let packet = SensorPacket::new(data.payload());

        let header = " ".repeat(14) + "Accel" + &" ".repeat(28) + "Gyro" + &" ".repeat(29) + "Mag";
        println!("{header}");

        let mut data_s = String::new();
        for field in packet.fields() {
            match field {
                Ok(field) => match field {
                    SensorField::ScaledAccel(accel) => {
                        data_s += &format!("{} | ", vector3_str(accel.scaled_accel));
                    }
                    SensorField::ScaledGyro(gyro) => {
                        data_s += &format!("{} | ", vector3_str(gyro.scaled_gyro));
                    }
                    SensorField::ScaledMag(mag) => {
                        data_s += &format!("{}", vector3_str(mag.scaled_mag));
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
