pub mod filter;
pub mod parse;
pub mod sensor;
pub mod shared;
mod types;

pub use crate::errors::ParseError;
pub use parse::ReadBuf;

pub use types::*;

/// helper function for validating length
#[inline]
fn ensure_len(buf: &&[u8], need: usize, descriptor: (u8, u8)) -> Result<(), ParseError> {
    if buf.len() < need {
        return Err(ParseError::LenTooShort {
            descriptor_set: descriptor.0,
            descriptor: descriptor.1,
            need,
            got: buf.len(),
        });
    }
    Ok(())
}
