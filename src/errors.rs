//! Error types

use thiserror::Error;

/// Error returned by Parser
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FrameError {
    /// A CRC mismatch occurred
    #[error("A CRC mismatch occurred")]
    CrcMismatch,
    /// Expected a SYNC value and got something else
    #[error("Expected a SYNC value and got something else")]
    UnexpectedByte,
}

/// Error when parsing MIP packet fields
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParseError {
    /// Not enough bytes in provided data to parse the field
    #[error(
        "Length of data didn't match expected for 0x{descriptor_set:x}:0x{descriptor:x}. Need: {need}. Got: {got}"
    )]
    LenTooShort {
        /// The descriptor set of the field triggering the error
        descriptor_set: u8,
        /// The descriptor of the field triggering the error
        descriptor: u8,
        /// The number of bytes required
        need: usize,
        /// The number of bytes available
        got: usize,
    },
    /// Encountered an unrecognized descriptor
    #[error("Unexpected descriptor 0x{descriptor:x} in set 0x{descriptor_set:x}")]
    UnknownField {
        /// The descriptor set of the packet
        descriptor_set: u8,
        /// The unrecognized descriptor
        descriptor: u8,
    },
    /// packet with an unrecognized descriptor set
    #[error("Unrecognized descriptor set 0x{descriptor_set:x}")]
    UnknownDescriptorSet {
        /// The unrecognized descriptor set
        descriptor_set: u8,
    },
    /// unexpected field in command response when an AckNack was expected
    #[error("Expected an AckNack field, found 0x{descriptor:x} instead.")]
    MissingAck {
        /// The descriptor of the unexpected field
        descriptor: u8,
    },
    /// field length less than 2
    #[error("A field had a length < 2, which is malformed")]
    InvalidFieldLength,
}
