// src/mip/data/parse.rs
use super::Error;
use core::convert::TryInto;

#[inline]
pub fn need(bytes: &[u8], descriptor_set: u8, descriptor: u8, need: usize) -> Result<(), Error> {
    if bytes.len() < need {
        return Err(Error::LenTooShort {
            descriptor_set,
            descriptor,
            need,
            got: bytes.len(),
        });
    }
    Ok(())
}

/// Read primitives from a byte slice, advancing the slice as you read.
pub trait ReadBuf {
    fn remaining(&self) -> usize;

    fn read_u8(&mut self) -> u8;
    fn read_i8(&mut self) -> i8;

    fn read_u16(&mut self) -> u16;
    fn read_i16(&mut self) -> i16;

    fn read_u32(&mut self) -> u32;
    fn read_i32(&mut self) -> i32;

    fn read_u64(&mut self) -> u64;
    fn read_i64(&mut self) -> i64;

    fn read_f32(&mut self) -> f32;
    fn read_f64(&mut self) -> f64;

    fn read_bytes<const N: usize>(&mut self) -> [u8; N];
}

impl ReadBuf for &[u8] {
    #[inline]
    fn remaining(&self) -> usize {
        self.len()
    }

    #[inline]
    fn read_u8(&mut self) -> u8 {
        let v = self[0];
        *self = &self[1..];
        v
    }

    #[inline]
    fn read_i8(&mut self) -> i8 {
        self.read_u8() as i8
    }

    #[inline]
    fn read_u16(&mut self) -> u16 {
        let v = u16::from_le_bytes(self[..2].try_into().unwrap());
        *self = &self[2..];
        v
    }

    #[inline]
    fn read_i16(&mut self) -> i16 {
        let v = i16::from_le_bytes(self[..2].try_into().unwrap());
        *self = &self[2..];
        v
    }

    #[inline]
    fn read_u32(&mut self) -> u32 {
        let v = u32::from_le_bytes(self[..4].try_into().unwrap());
        *self = &self[4..];
        v
    }

    #[inline]
    fn read_i32(&mut self) -> i32 {
        let v = i32::from_le_bytes(self[..4].try_into().unwrap());
        *self = &self[4..];
        v
    }

    #[inline]
    fn read_u64(&mut self) -> u64 {
        let v = u64::from_le_bytes(self[..8].try_into().unwrap());
        *self = &self[8..];
        v
    }

    #[inline]
    fn read_i64(&mut self) -> i64 {
        let v = i64::from_le_bytes(self[..8].try_into().unwrap());
        *self = &self[8..];
        v
    }

    #[inline]
    fn read_f32(&mut self) -> f32 {
        let v = f32::from_le_bytes(self[..4].try_into().unwrap());
        *self = &self[4..];
        v
    }

    #[inline]
    fn read_f64(&mut self) -> f64 {
        let v = f64::from_le_bytes(self[..8].try_into().unwrap());
        *self = &self[8..];
        v
    }

    #[inline]
    fn read_bytes<const N: usize>(&mut self) -> [u8; N] {
        let v: [u8; N] = self[..N].try_into().unwrap();
        *self = &self[N..];
        v
    }
}

#[derive(Debug, Clone)]
pub struct RawPayload<'a> {
    pub descriptor_set: u8,
    pub descriptor: u8,
    pub payload: &'a [u8],
}

impl<'a> RawPayload<'a> {
    pub fn new(descriptor_set: u8, descriptor: u8, payload: &'a [u8]) -> Self {
        Self {
            descriptor_set,
            descriptor,
            payload,
        }
    }
}
