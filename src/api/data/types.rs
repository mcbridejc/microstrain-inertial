use crate::api::{
    Error,
    data::{ReadBuf as _, ensure_len},
};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vector3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3f {
    pub const LEN: usize = 12;

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub const fn from_array(array: [f32; 3]) -> Self {
        Self {
            x: array[0],
            y: array[1],
            z: array[2],
        }
    }

    #[inline]
    pub fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, Error> {
        ensure_len(buf, Self::LEN, descriptor)?;
        Ok(Self {
            x: buf.read_f32(),
            y: buf.read_f32(),
            z: buf.read_f32(),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vector3d {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3d {
    pub const LEN: usize = 24;

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub const fn from_array(array: [f64; 3]) -> Self {
        Self {
            x: array[0],
            y: array[1],
            z: array[2],
        }
    }

    #[inline]
    pub fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, Error> {
        ensure_len(buf, Self::LEN, descriptor)?;
        Ok(Self {
            x: buf.read_f64(),
            y: buf.read_f64(),
            z: buf.read_f64(),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Matrix3f {
    /// Row-major: m[r][c] => data[r*3 + c]
    pub data: [f32; 9],
}

impl Matrix3f {
    /// The number of bytes required to encode a Matrix3f
    pub const LEN: usize = 9 * 4;

    pub const fn from_array(array: [f32; 9]) -> Self {
        Self { data: array }
    }

    #[inline]
    pub fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, Error> {
        ensure_len(buf, 36, descriptor)?;
        let mut data = [0.0f32; 9];
        for v in &mut data {
            *v = buf.read_f32();
        }
        Ok(Self { data })
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Quatf {
    /// (w, x, y, z)
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quatf {
    pub const LEN: usize = 16;

    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    pub const fn from_array(array: [f32; 4]) -> Self {
        Self {
            w: array[0],
            x: array[1],
            y: array[2],
            z: array[3],
        }
    }

    #[inline]
    pub fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, Error> {
        ensure_len(buf, 16, descriptor)?;
        Ok(Self {
            w: buf.read_f32(),
            x: buf.read_f32(),
            y: buf.read_f32(),
            z: buf.read_f32(),
        })
    }
}
