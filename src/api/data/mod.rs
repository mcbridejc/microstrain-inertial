pub mod filter;
pub mod parse;
pub mod sensor;
pub mod shared;

pub use super::Error;
pub use parse::ReadBuf;


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

#[derive(Debug, Copy, Clone)]
pub struct Vector3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3f {
    pub const LEN: usize = 12;

    #[inline]
    fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, Error> {
        ensure_len(buf, Self::LEN, descriptor)?;
        Ok(Self {
            x: buf.read_f32(),
            y: buf.read_f32(),
            z: buf.read_f32(),
        })
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Vector3d {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
impl Vector3d {
    pub const LEN: usize = 24;

    #[inline]
    fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, Error> {
        ensure_len(buf, Self::LEN, descriptor)?;
        Ok(Self {
            x: buf.read_f64(),
            y: buf.read_f64(),
            z: buf.read_f64(),
        })
    }
}


#[derive(Debug, Copy, Clone)]
pub struct Matrix3f {
    /// Row-major: m[r][c] => data[r*3 + c]
    pub data: [f32; 9],
}

impl Matrix3f {
    /// The number of bytes required to encode a Matrix3f
    pub const LEN: usize = 9 * 4;
    #[inline]
    fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, Error> {
        ensure_len(buf, 36, descriptor)?;
        let mut data = [0.0f32; 9];
        for v in &mut data {
            *v = buf.read_f32();
        }
        Ok(Self { data })
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Quatf {
    /// (w, x, y, z)
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quatf {
    const LEN: usize = 16;
    #[inline]
    fn read_from(buf: &mut &[u8], descriptor: (u8, u8)) -> Result<Self, Error> {
        ensure_len(buf, 16, descriptor)?;
        Ok(Self {
            w: buf.read_f32(),
            x: buf.read_f32(),
            y: buf.read_f32(),
            z: buf.read_f32(),
        })
    }
}