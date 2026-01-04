pub mod filter;
pub mod parse;
pub mod sensor;
pub mod shared;
mod types;

use std::marker::PhantomData;

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

trait FieldParse: Sized {
    const DESCRIPTOR_SET: u8;

    fn parse(descriptor: u8, payload: &[u8]) -> Result<Self, Error>;
}

pub struct FieldIter<'a, T> {
    remaining: &'a [u8],
    _marker: PhantomData<T>,
}

impl<'a, T: FieldParse> Iterator for FieldIter<'a, T> {
    type Item = Result<T, Error>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining.is_empty() {
            return None;
        }

        if self.remaining.len() < 2 {
            let err = Error::LenTooShort {
                descriptor_set: T::DESCRIPTOR_SET,
                descriptor: 0,
                need: 2,
                got: self.remaining.len(),
            };
            self.remaining = &[];
            return Some(Err(err));
        }

        let buf = self.remaining;
        let field_len = buf[0] as usize;
        let descriptor = buf[1];

        if buf.len() < field_len {
            let err = Error::LenTooShort {
                descriptor_set: T::DESCRIPTOR_SET,
                descriptor,
                need: field_len,
                got: buf.len(),
            };
            self.remaining = &[];
            return Some(Err(err));
        }

        let (field, rest) = buf.split_at(field_len);
        self.remaining = rest;
        let payload = &field[2..]; // skip len and descriptor bytes
        Some(T::parse(descriptor, payload))
    }
}
