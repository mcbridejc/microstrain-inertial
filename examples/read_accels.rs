//! Setup the IMU to output accels and print them

use std::{sync::Arc, time::Duration};

use clap::Parser;

use microstrain_inertial::{
    api::{
        commands::imu_3dm::DescriptorRate,
        data::sensor::{self, ScaledAccel, ScaledGyro, SensorField, SensorPacket},
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

    let descriptor_set = sensor::SENSOR_DESCRIPTOR_SET;
    let descriptor_rates = &[DescriptorRate {
        descriptor: sensor::ScaledAccel::DESCRIPTOR,
        decimation: 500,
    }];

    tokio::time::timeout(
        Duration::from_millis(500),
        interface.set_message_format(descriptor_set, descriptor_rates),
    )
    .await
    .ok();
    // .expect("Timedout waiting for set_message_format response")
    // .expect("Error setting rates");

    loop {
        let data = interface.read_raw_data().await.unwrap();
        let packet = SensorPacket::new(data.payload());
        for field in packet.fields() {
            match field {
                Ok(field) => match field {
                    SensorField::ScaledAccel(accel) => println!("{:?}", accel.scaled_accel),
                    _ => (),
                },
                Err(e) => {
                    log::error!("Error parsing sensor field: {e:?}");
                }
            }
        }
    }
}
