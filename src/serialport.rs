//! Background IO to serialport

use std::{sync::Arc, time::Duration};

use serial2::SerialPort;

use crate::{framer::MessageFramer, interface::AsyncInterface};

pub fn start_serialport_thread<const DATA_BUFFER_SIZE: usize>(
    port: &str,
    baud: u32,
    interface: Arc<AsyncInterface<DATA_BUFFER_SIZE>>,
) -> Result<(), std::io::Error> {
    let port = Arc::new(SerialPort::open(port, baud)?);

    let interface = Arc::new(interface);

    let tx_if = interface.clone();
    let tx_port = port.clone();
    std::thread::Builder::new()
        .name("microstrain tx".into())
        .spawn(move || {
            loop {
                let mut tx_buf = [0; 256];
                let cnt = tx_if.read_outgoing_bytes(&mut tx_buf);
                if cnt > 0 {
                    if let Err(e) = tx_port.write_all(&tx_buf[..cnt]) {
                        log::error!("Error writing to serial port: {:?}", e);
                    }
                }
                // TODO: A wakeup signal from the interface would be good...
                std::thread::sleep(Duration::from_millis(10));
            }
        })
        .unwrap();

    let rx_if = interface.clone();
    std::thread::Builder::new()
        .name("microstrain rx".into())
        .spawn(move || {
            loop {
                let mut read_buf = [0; 256];
                let mut framer = MessageFramer::new();
                let result = port.read(&mut read_buf);
                if result.is_err() {
                    log::error!("Error reading serial port: {:?}", result.unwrap_err());
                    continue;
                }
                let cnt = result.unwrap();
                for b in &read_buf[..cnt] {
                    if let Ok(Some(frame)) = framer.push_byte(*b) {
                        rx_if.push_message(frame);
                    }
                }
            }
        })
        .unwrap();
    Ok(())
}
