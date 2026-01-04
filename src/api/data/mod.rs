pub mod filter;
pub mod parse;
pub mod sensor;
pub mod shared;
mod types;

pub use super::Error;
pub use parse::ReadBuf;

pub use types::*;

/// helper function for validating length
#[inline]
fn ensure_len(buf: &&[u8], need: usize, descriptor: (u8, u8)) -> Result<(), Error> {
    if buf.len() < need {
        return Err(Error::LenTooShort {
            descriptor_set: descriptor.0,
            descriptor: descriptor.1,
            need,
            got: buf.len(),
        });
    }
    Ok(())
}
