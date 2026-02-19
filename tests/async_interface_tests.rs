use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use microstrain_inertial::api::commands::base::Ping;
use microstrain_inertial::api::commands::{AckNack, CommandSerialize};
use microstrain_inertial::framer::RawPacket;
use microstrain_inertial::interface::ReadDataError;

#[test]
fn test_ping_command() {
    let interface =
        microstrain_inertial::interface::AsyncInterface::<1024>::new_with_data_buffer_size();

    // Get the ping future
    let mut future = pin!(interface.ping());

    // Poll it, and check that it is pending
    let mut cx = Context::from_waker(Waker::noop());
    let poll_result = future.as_mut().poll(&mut cx);
    assert!(poll_result.is_pending(), "Poll returned: {:?}", poll_result);

    // Read the transmit data, and check that it contains the expected ping command packet
    let mut write_buf = [0; 255];
    let packet_size = Ping {}.serialize_command(&mut write_buf).unwrap();
    let exp_bytes = &write_buf[..packet_size];
    let mut read_buf = [0u8; 100];
    let read_count = interface.read_outgoing_bytes(&mut read_buf);
    assert_eq!(exp_bytes, &read_buf[..read_count]);

    // Send back the ping response
    let ping_resp = [1, 4, 4, 241, 1, 0];
    interface.push_message(RawPacket::new(&ping_resp));

    // Poll the future again, this time it should have completed successfully
    let poll_result = future.poll(&mut cx);
    assert!(poll_result.is_ready());
    let Poll::Ready(result) = poll_result else {
        panic!("")
    };
    let response = result.expect("Future returned an error");
    assert_eq!(AckNack::Ack, response.acknack);
}

#[test]
fn test_read_raw_data() {
    let interface =
        microstrain_inertial::interface::AsyncInterface::<64>::new_with_data_buffer_size();
    assert!(!interface.is_data_available());

    let msg = [0x80, 4, 2, 0x04, 0xAA, 0xBB];
    interface.push_message(RawPacket::new(&msg));
    assert!(interface.is_data_available());

    let mut future = pin!(interface.read_raw_data());
    let mut cx = Context::from_waker(Waker::noop());
    let poll_result = future.as_mut().poll(&mut cx);
    let Poll::Ready(result) = poll_result else {
        panic!("expected ready")
    };
    let packet = result.expect("unexpected data read error");
    assert_eq!(packet.as_slice(), &[0x80, 4, 2, 0x04, 0xAA, 0xBB]);
    drop(packet);
    assert!(!interface.is_data_available());
}

#[test]
fn test_read_raw_data_overrun() {
    let interface =
        microstrain_inertial::interface::AsyncInterface::<6>::new_with_data_buffer_size();

    let msg = [0x80, 4, 2, 0x04, 0xAA, 0xBB];
    interface.push_message(RawPacket::new(&msg));
    interface.push_message(RawPacket::new(&msg));

    let mut future = pin!(interface.read_raw_data());
    let mut cx = Context::from_waker(Waker::noop());
    let poll_result = future.as_mut().poll(&mut cx);
    let Poll::Ready(result) = poll_result else {
        panic!("expected ready")
    };
    let Err(err) = result else {
        panic!("expected overrun error")
    };
    assert_eq!(err, ReadDataError::Overrun);
}
