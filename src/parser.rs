use thiserror::Error;

use crate::serialize::OwnedMessage;

const MAX_PAYLOAD: usize = 255;

/// Error returned by Parser
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum ParseError {
    #[error("A CRC mismatch occurred")]
    CrcMismatch,
    /// Thrown when expecting a SYNC value but got the wrong value
    #[error("Unexpected byte")]
    UnexpectedByte,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RawMessage<'a> {
    buf: &'a [u8],
}

impl RawMessage<'_> {
    pub fn payload(&self) -> &[u8] {
        &self.buf[2..2 + self.len() as usize]
    }

    pub fn descriptor_set(&self) -> u8 {
        self.buf[0]
    }

    pub fn len(&self) -> u8 {
        self.buf[1]
    }

    pub fn to_owned(&self) -> OwnedMessage {
        OwnedMessage::new(self.descriptor_set(), self.payload())
    }
}

pub struct MessageParser {
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

impl MessageParser {
    const SYNC1: u8 = 0x75;
    const SYNC2: u8 = 0x65;

    pub fn new() -> Self {
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
    pub fn try_pending_message(&mut self) -> Option<RawMessage<'_>> {
        self.consume_pending().map(|len| RawMessage {
            buf: &self.buf[..len],
        })
    }

    /// Push a new byte into the parser
    ///
    /// When a SYNC byte is expected and some other byte is received, a
    /// [`ParseError::UnexpectedByte`] is returned. When the CRC is found not to match, a
    /// [`ParseError::CrcMismatch`] is returned. In this case, the bytes which were previous parsed
    /// may contain valid messages, so they will be re-parsed starting from the original
    /// descriptor_set byte on the next call to push_byte. However, in order to read pending
    /// messages immediately without receiving a new byte, one can call
    /// [`MessageParser::try_pending_message`].
    pub fn push_byte(&mut self, b: u8) -> Result<Option<RawMessage<'_>>, ParseError> {
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
                return Ok(Some(RawMessage {
                    buf: &self.buf[..len],
                }));
            } else {
                // We've cleared the pending bytes. Parse as normal
                self.pending_bytes = None;
            }
        }

        match Self::push_byte_inner(&mut self.state, &mut self.buf, &mut self.pending_bytes, b) {
            Ok(Some(len)) => Ok(Some(RawMessage {
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
    ) -> Result<Option<usize>, ParseError> {
        match state {
            ParseState::Sync1 => {
                if b == Self::SYNC1 {
                    *state = ParseState::Sync2;
                    Ok(None)
                } else {
                    *state = ParseState::Sync1;
                    Err(ParseError::UnexpectedByte)
                }
            }
            ParseState::Sync2 => {
                if b == Self::SYNC2 {
                    *state = ParseState::Descriptor;
                    Ok(None)
                } else {
                    *state = ParseState::Sync1;
                    Err(ParseError::UnexpectedByte)
                }
            }
            ParseState::Descriptor => {
                buf[0] = b;
                *state = ParseState::Length;
                Ok(None)
            }
            ParseState::Length => {
                buf[1] = b;
                *state = ParseState::Payload(0);
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
                let mut chk16 = fletcher::Fletcher16::new();
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
                    Err(ParseError::CrcMismatch)
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::parser::{MessageParser, ParseError};

    #[test]
    fn test_parse_message() {
        let count_payload: Vec<u8> = (0u8..32).collect();
        let mut msg = crate::serialize::OwnedMessage::new(0x10, &count_payload);
        let mut parser = MessageParser::new();

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

    /// If an incorrect length is detected, it may consume multiple messages before reaching a CRC
    /// failure, and then it should go back and return the two messages without dropping any bytes
    #[test]
    fn test_double_message_after_failure() {
        let count_payload: Vec<u8> = (0u8..255).collect();
        let msg1 = crate::serialize::OwnedMessage::new(0x10, &count_payload[0..4]);
        let msg2 = crate::serialize::OwnedMessage::new(0x20, &count_payload[0..8]);
        let msg3 = crate::serialize::OwnedMessage::new(0x30, &count_payload);
        let mut parser = MessageParser::new();
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
                    ParseError::CrcMismatch => crc_errors += 1,
                    ParseError::UnexpectedByte => (),
                },
            }
        }

        assert_eq!(3, messages.len());
        assert_eq!(1, crc_errors);
    }
}
