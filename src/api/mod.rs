use thiserror::Error;

#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum Error {
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
    #[error("InvalidEnum")]
    InvalidEnum {
        descriptor_set: u8,
        descriptor: u8,
        raw: u64,
    },
}

pub mod data;
