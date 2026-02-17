use bbqueue::{
    BBQueue,
    prod_cons::stream::StreamGrantR,
    traits::{coordination::cs::CsCoord, notifier::polling::Polling, storage::Inline},
};

type Queue<const N: usize> = BBQueue<Inline<N>, CsCoord, Polling>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DataBufferError {
    InvalidPacket,
    PacketTooLarge,
    InsufficientSpace,
}

pub struct DataBuffer<const N: usize> {
    queue: Queue<N>,
}

impl<const N: usize> Default for DataBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> DataBuffer<N> {
    pub const fn new() -> Self {
        Self {
            queue: Queue::new(),
        }
    }

    pub fn push_packet(&self, descriptor_set: u8, payload: &[u8]) -> Result<(), DataBufferError> {
        if payload.len() > u8::MAX as usize {
            return Err(DataBufferError::PacketTooLarge);
        }

        let packet_len = payload.len() + 2;
        let prod = self.queue.stream_producer();
        let mut grant = prod
            .grant_exact(packet_len)
            .map_err(|_| DataBufferError::InsufficientSpace)?;
        grant[0] = descriptor_set;
        grant[1] = payload.len() as u8;
        grant[2..packet_len].copy_from_slice(payload);
        grant.commit(packet_len);
        Ok(())
    }

    pub fn push_raw_packet(&self, packet: &[u8]) -> Result<(), DataBufferError> {
        if !is_valid_raw_packet(packet) {
            return Err(DataBufferError::InvalidPacket);
        }

        let prod = self.queue.stream_producer();
        let mut grant = prod
            .grant_exact(packet.len())
            .map_err(|_| DataBufferError::InsufficientSpace)?;
        grant.copy_from_slice(packet);
        grant.commit(packet.len());
        Ok(())
    }

    pub fn read_packet(&self) -> Option<DataPacketGuard<'_, N>> {
        let cons = self.queue.stream_consumer();
        let grant = cons.read().ok()?;
        let bytes =
            complete_packet_prefix_len(&grant[..], grant.len()).min(packet_len(&grant[..])?);
        if bytes < 2 {
            grant.release(0);
            return None;
        }
        Some(DataPacketGuard {
            grant: Some(grant),
            bytes,
        })
    }

    pub fn is_packet_available(&self) -> bool {
        let cons = self.queue.stream_consumer();
        let grant = match cons.read() {
            Ok(grant) => grant,
            Err(_) => return false,
        };
        let available = packet_len(&grant[..]).is_some_and(|len| len <= grant.len());
        grant.release(0);
        available
    }
}

pub struct DataPacketGuard<'a, const N: usize> {
    grant: Option<StreamGrantR<&'a Queue<N>>>,
    bytes: usize,
}

impl<const N: usize> DataPacketGuard<'_, N> {
    /// Get the full raw packet as a slice
    ///
    /// This includes the descriptor set byte, the payload len byte, and the paylod containing all fields
    pub fn as_slice(&self) -> &[u8] {
        &self.grant.as_ref().expect("missing read grant")[..self.bytes]
    }

    /// Get the payload for the data packet
    pub fn payload(&self) -> &[u8] {
        &self.as_slice()[2..]
    }

    /// Get the descriptor set for the packet
    pub fn descriptor_set(&self) -> u8 {
        self.as_slice()[0]
    }

    pub fn len(&self) -> usize {
        self.bytes
    }

    pub fn is_empty(&self) -> bool {
        self.bytes == 0
    }
}

impl<const N: usize> Drop for DataPacketGuard<'_, N> {
    fn drop(&mut self) {
        if let Some(grant) = self.grant.take() {
            grant.release(self.bytes);
        }
    }
}

/// Validate the the packet has an accurate length field
fn is_valid_raw_packet(packet: &[u8]) -> bool {
    if packet.len() < 2 {
        return false;
    }
    let payload_len = packet[1] as usize;
    if payload_len + 2 != packet.len() {
        return false;
    }
    true
}

fn packet_len(src: &[u8]) -> Option<usize> {
    if src.len() < 2 {
        return None;
    }
    let payload_len = src[1] as usize;
    let packet_len = payload_len + 2;
    Some(packet_len)
}

fn complete_packet_prefix_len(src: &[u8], max_len: usize) -> usize {
    let mut used = 0usize;
    let limit = src.len().min(max_len);

    while used < limit {
        let packet = &src[used..limit];
        let Some(packet_len) = packet_len(packet) else {
            break;
        };
        if used + packet_len > limit {
            break;
        }
        used += packet_len;
    }

    used
}

#[cfg(test)]
mod tests {
    use super::DataBuffer;

    fn assert_sync<T: Sync>() {}

    #[test]
    fn data_buffer_is_sync() {
        assert_sync::<DataBuffer<64>>();
    }

    #[test]
    fn read_packet_round_trip() {
        let db = DataBuffer::<32>::new();
        db.push_packet(0x80, &[2, 0x04, 0xAA, 0xBB]).unwrap();

        let packet = db.read_packet().unwrap();
        assert_eq!(packet.as_slice(), &[0x80, 4, 2, 0x04, 0xAA, 0xBB]);
    }

    #[test]
    fn packet_available_does_not_consume() {
        let db = DataBuffer::<64>::new();
        db.push_raw_packet(&[0x80, 4, 2, 0x01, 0xAA, 0xBB]).unwrap();
        assert!(db.is_packet_available());
        let packet = db.read_packet().unwrap();
        assert_eq!(packet.as_slice(), &[0x80, 4, 2, 0x01, 0xAA, 0xBB]);
        drop(packet);
        assert!(!db.is_packet_available());
    }

    #[test]
    fn read_packet_is_one_packet_only() {
        let db = DataBuffer::<64>::new();
        db.push_raw_packet(&[0x80, 3, 1, 0x01, 0x11]).unwrap();
        db.push_raw_packet(&[0x81, 3, 1, 0x02, 0x22]).unwrap();

        let p1 = db.read_packet().unwrap();
        assert_eq!(p1.as_slice(), &[0x80, 3, 1, 0x01, 0x11]);
        drop(p1);

        let p2 = db.read_packet().unwrap();
        assert_eq!(p2.as_slice(), &[0x81, 3, 1, 0x02, 0x22]);
    }
}
