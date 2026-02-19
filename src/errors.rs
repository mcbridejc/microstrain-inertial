use thiserror::Error;

/// Error returned by Parser
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FrameError {
    #[error("A CRC mismatch occurred")]
    CrcMismatch,
    #[error("Expected a SYNC value and got something else")]
    UnexpectedByte,
}

/// Error when parsing MIP packet fields
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParseError {
    #[error(
        "Length of data didn't match expected for 0x{descriptor_set:x}:0x{descriptor:x}. Need: {need}. Got: {got}"
    )]
    LenTooShort {
        descriptor_set: u8,
        descriptor: u8,
        need: usize,
        got: usize,
    },
    #[error("Unexpected descriptor 0x{descriptor:x} in set 0x{descriptor_set:x}")]
    UnknownField { descriptor_set: u8, descriptor: u8 },
    #[error("Unrecognized descriptor set 0x{descriptor_set:x}")]
    UnknownDescriptorSet { descriptor_set: u8 },
    #[error("Expected an AckNack field, found 0x{descriptor:x} instead.")]
    MissingAck { descriptor: u8 },
}
