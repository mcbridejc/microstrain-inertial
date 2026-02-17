use std::{sync::Arc, time::Duration};

use clap::Parser;
use serialport::{SerialPort, TTYPort};

use microstrain_inertial::{api::commands::base::{BasePacket, Ping}, interface::AsyncInterface, serialport::start_serialport_thread};

#[derive(Parser)]
struct Args {
    port: String,
}


#[tokio::main(flavor = "current_thread")]
async fn main() {
    let args = Args::parse();   

    let interface = Arc::new(AsyncInterface::new());

    // Create background serial port IO which will move data between the interface and serial port
    start_serialport_thread(&args.port, 115200, interface.clone()).expect("Failed to launch IO threads");

    let resp = interface.ping().await.expect("Error waiting for ping response");

    println!("Ping resp: {:?}", resp.acknack);

}
