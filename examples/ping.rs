use std::time::Duration;

use clap::Parser;
use serialport::{SerialPort, TTYPort};

use microstrain_inertial::api::commands::base::{BasePacket, Ping};

#[derive(Parser)]
struct Args {
    port: String,
}

fn main() {
    let args = Args::parse();

    let mut port = serialport::new(args.port, 115200).open().unwrap();

    let mut packet_buf = [0u8; 256];
    let mut cmd = BasePacket::new(&mut packet_buf);

    cmd.add_field(&Ping {}).unwrap();
    cmd.add_field(&Ping {}).unwrap();

    println!("Sending cmd: {:?}", cmd.as_bytes());
    port.write_all(cmd.as_bytes()).unwrap();

    std::thread::sleep(Duration::from_millis(500));

    let cnt = port.read(&mut packet_buf).unwrap();

    println!("Returned: {:?}", &packet_buf[..cnt]);
}
