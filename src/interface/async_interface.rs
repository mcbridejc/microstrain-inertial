use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::{Context, Poll, Waker};

use critical_section::Mutex;

use crate::MAX_PACKET_SIZE;
use crate::api::commands::base::Ping;
use crate::api::commands::imu_3dm::{DescriptorRate, MessageFormat, MessageFormatResponse};
use crate::api::commands::{
    CommandField, CommandResponse, CommandResponseData, CommandResponseIter, CommandSerialize,
};
use crate::api::data::filter::FILTER_DESCRIPTOR_SET;
use crate::api::data::sensor::SENSOR_DESCRIPTOR_SET;
use crate::api::data::shared::SHARED_DESCRIPTOR_SET;
use crate::api::data::system::SYSTEM_DESCRIPTOR_SET;
use crate::framer::RawPacket;
use crate::interface::{
    CommandSendError, DataBuffer, DataBufferError, DataPacketGuard, ReadDataError,
};

/// The default size of the data buffer used by AsyncInterface
pub const DEFAULT_DATA_BUFFER_SIZE: usize = 1024;

enum CommandSlotState {
    Empty,
    Transmit {
        frame_size: usize,
        tx_pos: usize,
        descriptor_set: u8,
    },
    Error(CommandSendError),
    Completed {
        frame_size: usize,
    },
}

struct CommandSlot {
    frame_buf: [u8; MAX_PACKET_SIZE],
    waker: Option<Waker>,
    state: CommandSlotState,
}

impl CommandSlot {
    pub const fn new() -> Self {
        Self {
            frame_buf: [0; MAX_PACKET_SIZE],
            waker: None,
            state: CommandSlotState::Empty,
        }
    }

    pub fn busy(&self) -> bool {
        !matches!(self.state, CommandSlotState::Empty)
    }

    pub fn transmit(&mut self, cmd: &dyn CommandSerialize) -> Result<(), CommandSendError> {
        if self.busy() {
            return Err(CommandSendError::CommandInProgress);
        }
        let descriptor_set = cmd.descriptor_set();
        let frame_size = cmd.serialize_command(&mut self.frame_buf)?;
        self.state = CommandSlotState::Transmit {
            frame_size,
            descriptor_set,
            tx_pos: 0,
        };
        Ok(())
    }

    pub fn store_error(&mut self, e: CommandSendError) {
        self.state = CommandSlotState::Error(e);
        if let Some(waker) = self.waker.take() {
            waker.wake();
        }
    }

    pub fn read_transmit_bytes(&mut self, buf: &mut [u8]) -> usize {
        match &mut self.state {
            CommandSlotState::Transmit {
                frame_size,
                tx_pos,
                descriptor_set: _,
            } => {
                let read_len = buf.len().min(*frame_size - *tx_pos);
                buf[..read_len].copy_from_slice(&self.frame_buf[*tx_pos..*tx_pos + read_len]);
                *tx_pos += read_len;
                read_len
            }
            _ => 0,
        }
    }

    pub fn expected_descriptor_set(&self) -> Option<u8> {
        match self.state {
            CommandSlotState::Transmit { descriptor_set, .. } => Some(descriptor_set),
            _ => None,
        }
    }

    pub fn store_response(&mut self, resp_payload: &[u8]) {
        let len = resp_payload.len();
        self.frame_buf[..len].copy_from_slice(resp_payload);
        self.state = CommandSlotState::Completed { frame_size: len };
        if let Some(waker) = self.waker.take() {
            waker.wake();
        }
    }

    pub fn register_waker(&mut self, waker: &Waker) {
        if self.waker.as_ref().is_some_and(|w| w.will_wake(waker)) {
            return;
        }
        self.waker = Some(waker.clone());
    }

    pub fn get_response<R: CommandResponseData>(
        &mut self,
    ) -> Option<Result<CommandResponse<R>, CommandSendError>> {
        match core::mem::replace(&mut self.state, CommandSlotState::Empty) {
            CommandSlotState::Completed { frame_size } => {
                let response_payload = &self.frame_buf[..frame_size];
                let resp = CommandResponseIter::new(response_payload).next();
                let Some(resp) = resp else {
                    return Some(Err(CommandSendError::UnexpectedResponse));
                };
                let resp = match resp {
                    Ok(r) => r,
                    Err(e) => return Some(Err(e.into())),
                };

                let acknack = resp.acknack;
                let data = if let Some(data) = resp.data {
                    match R::from_data(data) {
                        Ok(parsed) => Some(parsed),
                        Err(e) => return Some(Err(e.into())),
                    }
                } else {
                    None
                };

                Some(Ok(CommandResponse { acknack, data }))
            }
            CommandSlotState::Error(e) => Some(Err(e)),
            other => {
                self.state = other;
                None
            }
        }
    }

    pub fn cancel(&mut self) {
        self.state = CommandSlotState::Empty;
    }
}

struct SharedState {
    pending_command: CommandSlot,
}

impl SharedState {
    const fn new() -> Self {
        Self {
            pending_command: CommandSlot::new(),
        }
    }

    fn queue_command(&mut self, cmd: &dyn CommandSerialize) {
        if let Err(e) = self.pending_command.transmit(cmd) {
            self.pending_command.store_error(e);
        }
    }
}

/// An IMU interface with an async API for sending commands and reading data stream from the IMU
///
/// ## Creation
///
/// An interface can be created with a default buffer size of [`DEFAULT_DATA_BUFFER_SIZE`]:
///
/// `AsyncInterface::new()`,
///
/// Or, iti can be created with a custom buffer size:
///
/// `AsyncInterface::<N>::new_with_data_buffer_size()`
///
/// ## Serial IO
///
/// The user application is responsible for reading and writing data to the interface using the
/// [`Self::read_outgoing_bytes`] and [`Self::push_message`] methods. These are thread safe, using
/// critical sections, and may be called from any context. When the `serialport` feature is enabled,
/// the [`start_serialport_thread`](crate::serialport::start_serialport_thread) function may be used
/// to launch background threads for IO.
///  
pub struct AsyncInterface<const DATA_BUFFER_SIZE: usize = DEFAULT_DATA_BUFFER_SIZE> {
    state: Mutex<RefCell<SharedState>>,
    data_buffer: DataBuffer<DATA_BUFFER_SIZE>,
    data_waker: Mutex<RefCell<Option<Waker>>>,
    data_overrun: AtomicBool,
}

impl<const DATA_BUFFER_SIZE: usize> AsyncInterface<DATA_BUFFER_SIZE> {
    /// Create a new interface with an arbitraty DATA_BUFFER_SIZE generic
    pub const fn new_with_data_buffer_size() -> Self {
        Self {
            state: Mutex::new(RefCell::new(SharedState::new())),
            data_buffer: DataBuffer::new(),
            data_waker: Mutex::new(RefCell::new(None)),
            data_overrun: AtomicBool::new(false),
        }
    }

    /// Check if there is a data packet avialable.
    pub fn is_data_available(&self) -> bool {
        self.data_buffer.is_packet_available()
    }

    /// Return a future to read data packets
    ///
    /// The future will pend until a data packet is available, at which point it will return a
    /// Result with a [`DataPacketGuard`] on success, or an error to indicate that the data buffer
    /// has been overrun and some data was lost.
    pub fn read_raw_data(&self) -> ReadRawDataFuture<'_, DATA_BUFFER_SIZE> {
        ReadRawDataFuture { interface: self }
    }

    /// Push a message recieved from teh device to the interface for processing
    pub fn push_message(&self, msg: RawPacket) {
        let descriptor_set = msg.descriptor_set();

        critical_section::with(|cs| {
            let mut state = self.state.borrow_ref_mut(cs);
            let pending = &mut state.pending_command;
            if Some(descriptor_set) == pending.expected_descriptor_set() {
                pending.store_response(msg.payload());
            }
        });

        if is_data_descriptor_set(descriptor_set) {
            match self.data_buffer.push_packet(descriptor_set, msg.payload()) {
                Ok(()) => self.wake_data_reader(),
                Err(DataBufferError::InsufficientSpace) => {
                    self.data_overrun.store(true, Ordering::Release);
                    self.wake_data_reader();
                }
                Err(DataBufferError::InvalidPacketLength) => {
                    log::warn!(
                        "Dropped invalid data packet for descriptor set {descriptor_set:#04x}"
                    );
                }
                Err(DataBufferError::PacketTooLarge) => {
                    log::warn!(
                        "Dropped oversized data packet for descriptor set {descriptor_set:#04x}"
                    );
                }
            }
        }
    }

    /// Send a ping command to the device and await its response
    pub fn ping(
        &self,
    ) -> CommandResponseFuture<'_, <Ping as CommandField>::Response, DATA_BUFFER_SIZE> {
        self.send_command_field(&Ping {})
    }

    /// Set the message format for the given descriptor set
    ///
    /// This specifies which data fields will be transmitted, and at what rate. Data fields will be
    /// combined into a single packet, to the extend possible, but if too large, may extend into
    /// multiple packets.
    pub fn set_message_format(
        &self,
        descriptor_set: u8,
        descriptors: &[DescriptorRate],
    ) -> CommandResponseFuture<'_, (), DATA_BUFFER_SIZE> {
        let cmd = MessageFormat::Write {
            descriptor_set,
            descriptors,
        };
        let commands: &[&dyn CommandField<Response = MessageFormatResponse>] = &[&cmd];
        critical_section::with(|cs| {
            let mut state = self.state.borrow_ref_mut(cs);
            state.queue_command(&commands);
        });
        CommandResponseFuture::new(self)
    }

    /// Read transmit bytes ready to be sent by the interface
    pub fn read_outgoing_bytes(&self, buf: &mut [u8]) -> usize {
        critical_section::with(|cs| {
            let mut state = self.state.borrow_ref_mut(cs);
            state.pending_command.read_transmit_bytes(buf)
        })
    }

    /// Send an arbitrary command field, and get a future awaiting its response
    ///
    /// Any type which implements the [`CommandField`] trait can be sent as a command. The response
    /// is parsed using the `C::Response` type, which must implement [`CommandResponseData`] to
    /// parse the returned data. For commands which do not expect any returned data, this may be `()`.
    ///
    /// All commands will return an AckNack field.
    pub fn send_command_field<R, C>(
        &self,
        command: &C,
    ) -> CommandResponseFuture<'_, R, DATA_BUFFER_SIZE>
    where
        R: CommandResponseData,
        C: CommandField<Response = R>,
    {
        let commands: &[&dyn CommandField<Response = R>] = &[command];
        critical_section::with(|cs| {
            let mut state = self.state.borrow_ref_mut(cs);
            state.queue_command(&commands);
        });
        CommandResponseFuture::new(self)
    }

    fn register_data_waker(&self, waker: &Waker) {
        critical_section::with(|cs| {
            let mut slot = self.data_waker.borrow_ref_mut(cs);
            if slot.as_ref().is_some_and(|w| w.will_wake(waker)) {
                return;
            }
            *slot = Some(waker.clone());
        });
    }

    fn clear_data_waker(&self) {
        critical_section::with(|cs| {
            let mut slot = self.data_waker.borrow_ref_mut(cs);
            *slot = None;
        });
    }

    fn wake_data_reader(&self) {
        critical_section::with(|cs| {
            let mut slot = self.data_waker.borrow_ref_mut(cs);
            if let Some(waker) = slot.take() {
                waker.wake();
            }
        });
    }
}

impl<const DATA_BUFFER_SIZE: usize> Default for AsyncInterface<DATA_BUFFER_SIZE> {
    fn default() -> Self {
        Self::new_with_data_buffer_size()
    }
}

impl AsyncInterface<DEFAULT_DATA_BUFFER_SIZE> {
    /// Create a new AsyncInterface with the default buffer size
    pub const fn new() -> Self {
        Self::new_with_data_buffer_size()
    }
}

/// A future for returning data packets
pub struct ReadRawDataFuture<'a, const DATA_BUFFER_SIZE: usize> {
    interface: &'a AsyncInterface<DATA_BUFFER_SIZE>,
}

impl<'a, const DATA_BUFFER_SIZE: usize> Future for ReadRawDataFuture<'a, DATA_BUFFER_SIZE> {
    type Output = Result<DataPacketGuard<'a, DATA_BUFFER_SIZE>, ReadDataError>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let interface = self.interface;

        if interface.data_overrun.swap(false, Ordering::AcqRel) {
            return Poll::Ready(Err(ReadDataError::Overrun));
        }
        if let Some(packet) = interface.data_buffer.read_packet() {
            return Poll::Ready(Ok(packet));
        }

        interface.register_data_waker(cx.waker());

        if interface.data_overrun.swap(false, Ordering::AcqRel) {
            interface.clear_data_waker();
            return Poll::Ready(Err(ReadDataError::Overrun));
        }
        if let Some(packet) = interface.data_buffer.read_packet() {
            interface.clear_data_waker();
            return Poll::Ready(Ok(packet));
        }

        Poll::Pending
    }
}

/// Future to await the response to a command packet
pub struct CommandResponseFuture<'a, R, const DATA_BUFFER_SIZE: usize> {
    interface: &'a AsyncInterface<DATA_BUFFER_SIZE>,
    done: bool,
    _marker: core::marker::PhantomData<R>,
}

impl<'a, R, const DATA_BUFFER_SIZE: usize> CommandResponseFuture<'a, R, DATA_BUFFER_SIZE> {
    fn new(interface: &'a AsyncInterface<DATA_BUFFER_SIZE>) -> Self {
        Self {
            interface,
            done: false,
            _marker: Default::default(),
        }
    }
}

impl<R, const DATA_BUFFER_SIZE: usize> Future for CommandResponseFuture<'_, R, DATA_BUFFER_SIZE>
where
    R: CommandResponseData + Unpin,
{
    type Output = Result<CommandResponse<R>, CommandSendError>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();
        if this.done {
            panic!("polled CommandResponseFuture after completion");
        }

        critical_section::with(|cs| {
            let mut state = this.interface.state.borrow_ref_mut(cs);

            if let Some(resp) = state.pending_command.get_response() {
                this.done = true;
                Poll::Ready(resp)
            } else {
                state.pending_command.register_waker(cx.waker());
                Poll::Pending
            }
        })
    }
}

impl<R, const DATA_BUFFER_SIZE: usize> Drop for CommandResponseFuture<'_, R, DATA_BUFFER_SIZE> {
    fn drop(&mut self) {
        if self.done {
            return;
        }
        critical_section::with(|cs| {
            let mut state = self.interface.state.borrow_ref_mut(cs);
            state.pending_command.cancel();
        })
    }
}

fn is_data_descriptor_set(descriptor_set: u8) -> bool {
    matches!(
        descriptor_set,
        SENSOR_DESCRIPTOR_SET
            | FILTER_DESCRIPTOR_SET
            | SHARED_DESCRIPTOR_SET
            | SYSTEM_DESCRIPTOR_SET
    )
}
