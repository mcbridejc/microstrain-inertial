//! MIP Packet Framer/Parser
use crate::{checksum::Checksum, errors::FrameError, owned_packet::OwnedPacket};

const MAX_PAYLOAD: usize = 255;

#[allow(clippy::len_without_is_empty)]
#[derive(Clone, Copy, Debug, PartialEq)]
/// Wrapper for a MIP packet buffer
pub struct RawPacket<'a> {
    buf: &'a [u8],
}

impl<'a> RawPacket<'a> {
    /// Create a new raw message from a packet buffer
    ///
    /// `buf` should not include SYNC bytes and must have a valid length field which matches the
    /// size of the slice.
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf }
    }

    /// Return the payload of the packet
    pub fn payload(&self) -> &[u8] {
        let end = (2 + self.len() as usize).min(self.buf.len());
        &self.buf[2..end]
    }

    /// Return the descriptor set for the packet
    pub fn descriptor_set(&self) -> u8 {
        self.buf[0]
    }

    /// Get the payload length as encoded in the packet
    pub fn len(&self) -> u8 {
        self.buf[1]
    }

    /// Convert the message into an OwnedMessage
    pub fn to_owned(&self) -> OwnedPacket {
        OwnedPacket::new_with_payload(self.descriptor_set(), self.payload())
    }
}

/// Parse incoming stream of bytes into MIP packets
///
/// Messages are transmitted with two sync bytes, a descriptor set byte, and a payload length byte
/// as header, following by the payload, following by two checksum bytes:
///
/// `<SYNC1> <SYNC2> <DESCRIPTOR_SET> <PAYLOAD_LEN(N)> <N payload bytes> <CHK_H> <CHK_L>`
///  
/// The framer searches for sync bytes, and when found parses the remaining bytes as a message. If a
/// CRC mismatch occurs, the parser will rewind and resume parsing one byte later than the previous
/// start, attempting again to find a valid packet.
///
/// When a rewind occurs due to malformed input, it is possible that when
/// [`push_byte`](Self::push_byte) returns a completed packet there remain more packets ready in the
/// un-parsed data buffer. If so, this packet will be returned on the next call to
/// [`push_byte`](Self::push_byte). In order to process these packets immediately, uses can call
/// [`try_pending_message`](Self::try_pending_message).
///
/// # Example
///
/// ```rust
/// use microstrain_inertial::{
///     framer::MessageFramer,
///     SYNC1, SYNC2,
/// };
///
/// let mut framer = MessageFramer::new();
/// let msg = [SYNC1, SYNC2, 10, 3, 1, 2, 3];
/// let crc = microstrain_inertial::checksum::calc_checksum(&msg);
///
/// for b in &msg {
///     let result = framer.push_byte(*b).expect("Error parsing byte");
///     assert!(result.is_none()); // No packet was completed
/// }
/// framer.push_byte((crc >> 8) as u8);
/// // Send last byte of frame, get back RawPacket
/// let frame = framer.push_byte(crc as u8).expect("Error parsing byte").expect("No packet was completed");
/// assert_eq!(10, frame.descriptor_set());
/// assert_eq!(&[1,2,3], frame.payload());
///
/// ```
///
pub struct MessageFramer {
    buf: [u8; MAX_PAYLOAD + 5],
    state: ParseState,
    pending_bytes: Option<(usize, usize)>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ParseState {
    Sync1,
    Sync2,
    Descriptor,
    Length,
    Payload(u8),
    CheckH,
    CheckL,
}

impl Default for MessageFramer {
    fn default() -> Self {
        Self::new()
    }
}

impl MessageFramer {
    const SYNC1: u8 = 0x75;
    const SYNC2: u8 = 0x65;

    /// Create a new MessageFramer
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_PAYLOAD + 5],
            state: ParseState::Sync1,
            pending_bytes: None,
        }
    }

    fn shift_buf(&mut self, n: usize) {
        for i in 0..self.buf.len() - n {
            self.buf[i] = self.buf[i + n];
        }
    }

    /// Attempt to parse a message out of the pending bytes
    ///
    /// Returns the size of the message in buf if one is found
    fn consume_pending(&mut self) -> Option<usize> {
        let (start, mut end) = self.pending_bytes?;

        if start != 0 {
            self.shift_buf(start);
        }

        end -= start;
        let mut i = 0;
        loop {
            let b = self.buf[i];
            i += 1;
            match Self::push_byte_inner(&mut self.state, &mut self.buf, &mut self.pending_bytes, b)
            {
                // Completed a message
                Ok(Some(len)) => {
                    // Store how many bytes are still pending after consuming this message
                    // They are not shifted now, because we have to return a reference to them
                    self.pending_bytes = if i < end { Some((i, end)) } else { None };
                    return Some(len);
                }
                // Still working on a valid message
                Ok(None) => (),
                // Failed, move the start pointer up one byte and start parsing again from there
                Err(_) => {
                    self.shift_buf(1);
                    end -= 1;
                    i = 0;
                }
            }
            if i == end {
                // We successfully consumed all pending bytes, but found no message. A partial
                // message will be properly reflected in self.state and awaiting new bytes to be
                // pushed
                self.pending_bytes = None;
                return None;
            }
        }
    }

    /// Attempt to read a message from the pending bytes
    ///
    /// If bytes are left pending after a CRC error, this method will consume those pending bytes
    /// until either a valid message is found or all pending bytes are consumed. Once None is
    /// returned by this message, no valid message can be return until `push_byte` is called to
    /// deliver more bytes to the parser.
    pub fn try_pending_message(&mut self) -> Option<RawPacket<'_>> {
        self.consume_pending().map(|len| RawPacket {
            buf: &self.buf[..len],
        })
    }

    /// Push a new byte into the parser
    ///
    /// When a SYNC byte is expected and some other byte is received, a
    /// [`FrameError::UnexpectedByte`] is returned. When the CRC is found not to match, a
    /// [`FrameError::CrcMismatch`] is returned. In this case, the bytes which were previous parsed
    /// may contain valid messages, so they will be re-parsed starting from the original
    /// descriptor_set byte on the next call to push_byte. However, in order to read pending
    /// messages immediately without receiving a new byte, one can call
    /// [`MessageFramer::try_pending_message`].
    pub fn push_byte(&mut self, b: u8) -> Result<Option<RawPacket<'_>>, FrameError> {
        if self.pending_bytes.is_some() {
            if let Some(len) = self.consume_pending() {
                // We found a message in the pending bytes.
                // Add the provided byte onto the pending bytes and return the found message
                if let Some((start, end)) = self.pending_bytes {
                    self.buf[end] = b;
                    self.pending_bytes = Some((start, end + 1));
                } else {
                    self.buf[0] = b;
                    self.pending_bytes = Some((0, 1));
                }
                return Ok(Some(RawPacket {
                    buf: &self.buf[..len],
                }));
            } else {
                // We've cleared the pending bytes. Parse as normal
                self.pending_bytes = None;
            }
        }

        match Self::push_byte_inner(&mut self.state, &mut self.buf, &mut self.pending_bytes, b) {
            Ok(Some(len)) => Ok(Some(RawPacket {
                buf: &self.buf[0..len],
            })),
            Ok(None) => Ok(None),
            Err(e) => Err(e),
        }
    }

    fn push_byte_inner(
        state: &mut ParseState,
        buf: &mut [u8],
        pending_bytes: &mut Option<(usize, usize)>,
        b: u8,
    ) -> Result<Option<usize>, FrameError> {
        match state {
            ParseState::Sync1 => {
                if b == Self::SYNC1 {
                    *state = ParseState::Sync2;
                    Ok(None)
                } else {
                    *state = ParseState::Sync1;
                    Err(FrameError::UnexpectedByte)
                }
            }
            ParseState::Sync2 => {
                if b == Self::SYNC2 {
                    *state = ParseState::Descriptor;
                    Ok(None)
                } else {
                    *state = ParseState::Sync1;
                    Err(FrameError::UnexpectedByte)
                }
            }
            ParseState::Descriptor => {
                buf[0] = b;
                *state = ParseState::Length;
                Ok(None)
            }
            ParseState::Length => {
                buf[1] = b;
                if b != 0 {
                    *state = ParseState::Payload(0);
                } else {
                    // If there are 0 payload bytes, go to expecting checksum
                    *state = ParseState::CheckH;
                }

                Ok(None)
            }
            ParseState::Payload(i) => {
                let payload_len = buf[1];
                buf[*i as usize + 2] = b;
                if *i + 1 == payload_len {
                    *state = ParseState::CheckH;
                    Ok(None)
                } else {
                    *state = ParseState::Payload(*i + 1);
                    Ok(None)
                }
            }
            ParseState::CheckH => {
                let payload_len = buf[1];
                buf[payload_len as usize + 2] = b;
                *state = ParseState::CheckL;
                Ok(None)
            }
            ParseState::CheckL => {
                let payload_len = buf[1] as usize;
                buf[payload_len + 3] = b;
                let mut chk16 = Checksum::new();
                chk16.update(&[Self::SYNC1, Self::SYNC2]);
                chk16.update(&buf[..payload_len + 2]);

                let message_chk =
                    ((buf[payload_len + 2] as u16) << 8) | buf[payload_len + 3] as u16;
                if chk16.value() == message_chk {
                    *state = ParseState::Sync1;
                    Ok(Some(payload_len + 2))
                } else {
                    *state = ParseState::Sync1;
                    *pending_bytes = Some((0, payload_len + 4));
                    // Return the CRC error for now. Future bytes will be scanned on next call.
                    Err(FrameError::CrcMismatch)
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::framer::{FrameError, MessageFramer};

    #[test]
    fn test_parse_message() {
        let count_payload: Vec<u8> = (0u8..32).collect();
        let mut msg = crate::owned_packet::OwnedPacket::new_with_payload(0x10, &count_payload);
        let mut parser = MessageFramer::new();

        let mut parsed = None;
        for (i, b) in msg.as_slice().iter().enumerate() {
            match parser.push_byte(*b) {
                Ok(Some(m)) => parsed = Some(m.to_owned()),
                Ok(None) => (), // expected
                Err(e) => panic!("Got error parsing byte {i}: {e:?}"),
            }
        }

        let parsed = parsed.unwrap();
        assert_eq!(parsed.descriptor_set(), 0x10);
        assert_eq!(parsed.payload(), &count_payload);
    }

    #[test]
    fn test_zero_length_payload_frame_bug() {
        // A valid frame with descriptor set + zero-length payload + CRC.
        let mut msg = crate::owned_packet::OwnedPacket::new(0x10);
        let raw = msg.as_slice().to_vec();
        let mut parser = MessageFramer::new();

        let mut parsed_messages = Vec::new();
        let mut frame_errors = Vec::new();

        for b in raw {
            match parser.push_byte(b) {
                Ok(Some(m)) => parsed_messages.push(m.to_owned()),
                Ok(None) => (),
                Err(e @ (FrameError::CrcMismatch | FrameError::UnexpectedByte)) => {
                    frame_errors.push(e)
                }
            }
        }

        // This should parse as one message with an empty payload.
        assert_eq!(1, parsed_messages.len());
        assert_eq!(0, frame_errors.len());
        assert_eq!(0x10, parsed_messages[0].descriptor_set());
        assert!(parsed_messages[0].payload().is_empty());
    }

    /// If an incorrect length is detected, it may consume multiple messages before reaching a CRC
    /// failure, and then it should go back and return the two messages without dropping any bytes
    #[test]
    fn test_double_message_after_failure() {
        let count_payload: Vec<u8> = (0u8..255).collect();
        let msg1 = crate::owned_packet::OwnedPacket::new_with_payload(0x10, &count_payload[0..4]);
        let msg2 = crate::owned_packet::OwnedPacket::new_with_payload(0x20, &count_payload[0..8]);
        let msg3 = crate::owned_packet::OwnedPacket::new_with_payload(0x30, &count_payload);
        let mut parser = MessageFramer::new();
        // Intentially break the parser by making it expect as 128 byte message
        assert_eq!(Ok(None), parser.push_byte(0x75)); // SYNC1
        assert_eq!(Ok(None), parser.push_byte(0x65)); // SYNC2
        assert_eq!(Ok(None), parser.push_byte(0)); // Arbitrary descriptor set
        assert_eq!(Ok(None), parser.push_byte(128)); // length

        let mut crc_errors = 0;
        let mut messages = Vec::new();
        for byte in [msg1, msg2, msg3]
            .iter_mut()
            .map(|m| m.as_slice())
            .flatten()
        {
            match parser.push_byte(*byte) {
                Ok(Some(m)) => {
                    messages.push(m.to_owned());
                }
                Ok(None) => (),
                Err(e) => match e {
                    FrameError::CrcMismatch => crc_errors += 1,
                    FrameError::UnexpectedByte => (),
                },
            }
        }

        assert_eq!(3, messages.len());
        assert_eq!(1, crc_errors);
    }
}
