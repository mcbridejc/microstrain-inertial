use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use microstrain_inertial::api::commands::AckNack;
use microstrain_inertial::api::commands::base::{BasePacket, Ping};
use microstrain_inertial::framer::RawMessage;

#[test]
fn test_ping_command() {
    let interface = microstrain_inertial::interface::AsyncInterface::new(&[]);

    // Get the ping future
    let mut future = pin!(interface.ping());

    // Poll it, and check that it is pending
    let mut cx = Context::from_waker(Waker::noop());
    let poll_result = future.as_mut().poll(&mut cx);
    assert!(poll_result.is_pending(), "Poll returned: {:?}", poll_result);

    // Read the transmit data, and check that it contains the expected ping command packet
    let mut packet = BasePacket::from_fields(&[&Ping {}]).unwrap();
    let exp_bytes = packet.as_slice();
    let mut read_buf = [0u8; 100];
    let read_count = interface.read_outgoing_bytes(&mut read_buf);
    assert_eq!(exp_bytes, &read_buf[..read_count]);

    // Send back the ping response
    let ping_resp = [1, 4, 4, 241, 1, 0];
    interface.push_message(RawMessage::new(&ping_resp));

    // Poll the future again, this time it should have completed successfully
    let poll_result = future.poll(&mut cx);
    assert!(poll_result.is_ready());
    let Poll::Ready(result) = poll_result else {
        panic!("")
    };
    let response = result.expect("Future returned an error");
    assert_eq!(AckNack::Ack, response.acknack);
}
