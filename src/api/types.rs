//! Complex MIP data types

use crate::api::parse::ReadBuf as _;
use crate::errors::ParseError;

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

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
/// A 3D vector type using 32-bit floating point
pub struct Vector3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3f {
    /// The size of the vector when serialized in a packet
    pub const LEN: usize = 12;

    /// Create a new vector
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Create a new vector from array of values
    pub const fn from_array(array: [f32; 3]) -> Self {
        Self {
            x: array[0],
            y: array[1],
            z: array[2],
        }
    }

    #[inline]
    /// Read a vector from a packet buffer
    pub(crate) fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, ParseError> {
        ensure_len(buf, Self::LEN, descriptor)?;
        Ok(Self {
            x: buf.read_f32(),
            y: buf.read_f32(),
            z: buf.read_f32(),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
/// A 3D vector type using 64-bit floating point
pub struct Vector3d {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3d {
    /// The size of the vector when serialized to a packet
    pub const LEN: usize = 24;

    /// Create a new vector
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Create a vector from an array of values
    pub const fn from_array(array: [f64; 3]) -> Self {
        Self {
            x: array[0],
            y: array[1],
            z: array[2],
        }
    }

    #[inline]
    /// Read/consume a vector from a packet buffer
    pub(crate) fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, ParseError> {
        ensure_len(buf, Self::LEN, descriptor)?;
        Ok(Self {
            x: buf.read_f64(),
            y: buf.read_f64(),
            z: buf.read_f64(),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
/// A 3x3 matrix using 32-bit floating point
pub struct Matrix3f {
    /// Row-major: `m[r][c] => data[r*3 + c]`
    pub data: [f32; 9],
}

impl Matrix3f {
    /// The number of bytes required to encode a Matrix3f
    pub const LEN: usize = 9 * 4;

    /// Create a new matrix from an array of values
    pub const fn from_array(array: [f32; 9]) -> Self {
        Self { data: array }
    }

    #[inline]
    pub(crate) fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, ParseError> {
        ensure_len(buf, 36, descriptor)?;
        let mut data = [0.0f32; 9];
        for v in &mut data {
            *v = buf.read_f32();
        }
        Ok(Self { data })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
/// A quaternion using 32-bit floating point
pub struct Quatf {
    /// (w, x, y, z)
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quatf {
    /// The size of the quaternion when serialized into a packet
    pub const LEN: usize = 16;

    /// Create a new quaternion
    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    /// Create a new quaternion from an array of values
    pub const fn from_array(array: [f32; 4]) -> Self {
        Self {
            w: array[0],
            x: array[1],
            y: array[2],
            z: array[3],
        }
    }

    #[inline]
    pub(crate) fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, ParseError> {
        ensure_len(buf, 16, descriptor)?;
        Ok(Self {
            w: buf.read_f32(),
            x: buf.read_f32(),
            y: buf.read_f32(),
            z: buf.read_f32(),
        })
    }
}
