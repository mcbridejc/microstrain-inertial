//! Utils for parsing MIP fields in a packet payload

use core::marker::PhantomData;

use crate::errors::ParseError;

/// A trait for a type which can be constructed from MIP packet fields.
pub trait FieldParse: Sized {
    const DESCRIPTOR_SET: u8;

    fn parse(descriptor: u8, payload: &[u8]) -> Result<Self, ParseError>;
}

pub struct FieldIter<'a, T> {
    remaining: &'a [u8],
    _marker: PhantomData<T>,
}

impl<'a, T> FieldIter<'a, T> {
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            remaining: data,
            _marker: Default::default(),
        }
    }
}

impl<'a, T: FieldParse> Iterator for FieldIter<'a, T> {
    type Item = Result<T, ParseError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining.is_empty() {
            return None;
        }

        if self.remaining.len() < 2 {
            let err = ParseError::LenTooShort {
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
            let err = ParseError::LenTooShort {
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
