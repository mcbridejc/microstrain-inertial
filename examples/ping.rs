//! Setup the IMU to output accels and print them

use std::{sync::Arc, time::Duration};

use clap::Parser;

use microstrain_inertial::{interface::AsyncInterface, serialport::start_serialport_thread};

#[derive(Parser)]
struct Args {
    port: String,
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let args = Args::parse();

    let interface = Arc::new(AsyncInterface::new());

    // Create background serial port IO which will move data between the interface and serial port
    start_serialport_thread(&args.port, 115200, interface.clone())
        .expect("Failed to launch IO threads");

    let resp = tokio::time::timeout(Duration::from_millis(500), interface.ping())
        .await
        .expect("Timedout waiting for ping response")
        .expect("Error waiting for ping response");

    println!("Ping resp: {:?}", resp.acknack);
}
