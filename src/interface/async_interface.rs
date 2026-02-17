use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, Waker};

use critical_section::Mutex;
use thiserror::Error;

use crate::MAX_PACKET_SIZE;
use crate::api::commands::base::{BaseCommandField, BasePacket, Ping};
use crate::api::commands::{
    self, CommandField, CommandResponse, CommandResponseData, CommandResponseIter,
    CommandSerialize, SerializeError,
};
use crate::api::data::sensor::{SENSOR_DESCRIPTOR_SET, ScaledAccel, SensorField, SensorPacket};
use crate::errors::{FrameError, ParseError};
use crate::framer::{MessageParser, RawMessage};
use crate::serialize::OwnedMessage;

const MAX_COMMAND_FRAME_SIZE: usize = 260;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum AsyncError {
    #[error("command serialization failed")]
    SerializeFailed,
    #[error("no command waiter capacity")]
    CommandCapacity,
    #[error("no accelerometer waiter capacity")]
    AccelWaiterCapacity,
    #[error("waiter was cancelled")]
    Cancelled,
}

#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum CommandSendError {
    #[error("Command serialization failed")]
    SerializeFailed(#[from] SerializeError),
    #[error("An error occurred parsing the response to the command")]
    ResponseParseError(#[from] ParseError),
    #[error("Another command is still pending")]
    CommandInProgress,
    #[error("The received response did not match expectations")]
    UnexpectedResponse,
}

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
    pub fn new() -> Self {
        Self {
            frame_buf: [0; MAX_PACKET_SIZE],
            waker: None,
            state: CommandSlotState::Empty,
        }
    }

    pub fn busy(&self) -> bool {
        match self.state {
            CommandSlotState::Empty => false,
            _ => true,
        }
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
    }

    pub fn read_transmit_bytes(&mut self, buf: &mut [u8]) -> usize {
        match self.state {
            CommandSlotState::Transmit {
                frame_size,
                tx_pos,
                descriptor_set: _,
            } => {
                let read_len = buf.len().min(frame_size - tx_pos);
                buf[..read_len].copy_from_slice(&self.frame_buf[tx_pos..tx_pos + read_len]);
                read_len
            }
            _ => 0,
        }
    }

    /// The descriptor set for the in progress command
    ///
    /// None if no command is in progress
    pub fn expected_descriptor_set(&self) -> Option<u8> {
        match self.state {
            CommandSlotState::Transmit {
                frame_size: _,
                tx_pos: _,
                descriptor_set,
            } => Some(descriptor_set),
            _ => None,
        }
    }

    pub fn store_response(&mut self, resp_payload: &[u8]) {
        let len = resp_payload.len();
        self.frame_buf[..len].copy_from_slice(resp_payload);
        self.state = CommandSlotState::Completed { frame_size: len };
    }

    pub fn get_response<R: CommandResponseData>(
        &mut self,
    ) -> Option<Result<CommandResponse<R>, CommandSendError>> {
        match self.state {
            CommandSlotState::Completed { frame_size } => {
                let response_payload = &self.frame_buf[..frame_size];

                let resp = CommandResponseIter::new(&response_payload).next();
                if resp.is_none() {
                    return Some(Err(CommandSendError::UnexpectedResponse));
                }
                let resp = resp.unwrap();
                if let Err(e) = resp {
                    return Some(Err(e.into()));
                }
                let resp = resp.unwrap();

                let acknack = resp.acknack;
                let data = if let Some(data) = resp.data {
                    match R::from_data(data) {
                        Ok(data) => Some(data),
                        Err(e) => return Some(Err(e.into())),
                    }
                } else {
                    None
                };

                let resp = CommandResponse { acknack, data };

                Some(Ok(resp))
            }
            _ => None,
        }
    }

    /// Cancel a pending command
    pub fn cancel(&mut self) {
        self.state = CommandSlotState::Empty;
    }
}

struct CompletedCommand {
    id: u64,
    result: Result<OwnedMessage, AsyncError>,
}

pub trait DataWaiter: Send + Sync {
    fn id(&self) -> u64;
    fn wake(&self);
    fn handle_message(&self, msg: &OwnedMessage) -> bool;
}

// struct DataWaiter {
//     id: u64,
//     result: Option<OwnedMessage>,
//     waker: Option<Waker>,
// }

struct SharedState<'a> {
    pending_command: CommandSlot,
    data_waiters: &'a [&'a dyn DataWaiter],
}

impl<'a> SharedState<'a> {
    fn new(data_waiters: &'a [&'a dyn DataWaiter]) -> Self {
        Self {
            pending_command: CommandSlot::new(),
            data_waiters,
        }
    }

    fn queue_command(&mut self, cmd: &dyn CommandSerialize) {
        if let Err(e) = self.pending_command.transmit(cmd) {
            self.pending_command.store_error(e);
        }
    }

    fn store_completed_command(&mut self, response: &RawMessage) {
        self.pending_command.store_response(response.payload());
    }

    fn register_command_waker(&mut self, waker: &Waker) {
        self.pending_command.waker = Some(waker.clone());
    }

    // fn cancel_command(&mut self, waiter_id: u32) {
    //     if self
    //         .in_flight
    //         .as_ref()
    //         .is_some_and(|slot| slot.id == waiter_id)
    //     {
    //         self.in_flight = None;
    //         self.promote_queued_if_possible();
    //         return;
    //     }

    //     if self
    //         .queued
    //         .as_ref()
    //         .is_some_and(|slot| slot.id == waiter_id)
    //     {
    //         self.queued = None;
    //         return;
    //     }

    //     if let Some(idx) = self.find_completed_index(waiter_id) {
    //         self.completed[idx] = None;
    //     }
    // }

    // fn register_accel_waiter(&mut self, waiter_id: u64) -> Result<(), AsyncError> {
    //     for slot in &mut self.accel_waiters {
    //         if slot.is_none() {
    //             *slot = Some(AccelWaiter {
    //                 id: waiter_id,
    //                 result: None,
    //                 waker: None,
    //             });
    //             return Ok(());
    //         }
    //     }
    //     Err(AsyncError::AccelWaiterCapacity)
    // }

    // fn complete_data(&mut self, accel: ScaledAccel) {
    //     for waiter in self.data_waiters {
    //         if let Some(waiter) = waiter.as_mut().filter(|w| w.result.is_none()) {
    //             waiter.result = Some(accel);
    //             if let Some(waker) = waiter.waker.take() {
    //                 waker.wake();
    //             }
    //         }
    //     }
    // }

    // fn register_data_waker(&mut self, waiter_id: u64, waker: &Waker) -> bool {
    //     for waiter in &mut self.data {
    //         if let Some(waiter) = waiter.as_mut().filter(|w| w.id == waiter_id) {
    //             waiter.waker = Some(waker.clone());
    //             return true;
    //         }
    //     }
    //     false
    // }

    // fn take_accel_result(&mut self, waiter_id: u64) -> Option<ScaledAccel> {
    //     for waiter in &mut self.accel_waiters {
    //         if let Some(existing) = waiter.as_mut().filter(|w| w.id == waiter_id) {
    //             if let Some(result) = existing.result.take() {
    //                 *waiter = None;
    //                 return Some(result);
    //             }
    //             return None;
    //         }
    //     }
    //     None
    // }

    // fn cancel_accel_waiter(&mut self, waiter_id: u64) {
    //     for waiter in &mut self.accel_waiters {
    //         if waiter.as_ref().is_some_and(|w| w.id == waiter_id) {
    //             *waiter = None;
    //             return;
    //         }
    //     }
    // }
}

pub struct AsyncInterface<'a> {
    state: Mutex<RefCell<SharedState<'a>>>,
}

impl<'a> AsyncInterface<'a> {
    pub fn new(data_waiters: &'a [&'a dyn DataWaiter]) -> Self {
        Self {
            state: Mutex::new(RefCell::new(SharedState::new(data_waiters))),
        }
    }

    /// Push message
    pub fn push_message(&self, msg: RawMessage) {
        // TODO: Handle data messages
        // if msg.descriptor_set() == SENSOR_DESCRIPTOR_SET {
        //     let packet = SensorPacket::new(msg.payload());
        //     for field in packet.fields().flatten() {
        //         if let SensorField::ScaledAccel(accel) = field {
        //             state.complete_accel(accel);
        //         }
        //     }
        // }

        critical_section::with(|cs| {
            let mut state = self.state.borrow_ref_mut(cs);
            let pending = &mut state.pending_command;
            if Some(msg.descriptor_set()) == pending.expected_descriptor_set() {
                pending.store_response(msg.payload());
            }
        });
    }

    pub fn ping<'b: 'a>(&'b self) -> CommandResponseFuture<'b, <Ping as CommandField>::Response> {
        self.send_command_field(&Ping {})
    }

    pub fn read_outgoing_bytes(&self, buf: &mut [u8]) -> usize {
        critical_section::with(|cs| {
            let mut state = self.state.borrow_ref_mut(cs);
            state.pending_command.read_transmit_bytes(buf)
        })
    }

    pub fn send_command_field<'b: 'a, R, C>(&'b self, command: &C) -> CommandResponseFuture<'b, R>
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

    // pub fn next_scaled_accel<'a>(
    //     &'a self,
    // ) -> Result<NextScaledAccelFuture<'a, N_ACCEL_WAITERS>, AsyncError> {
    //     let mut state = self.state.borrow_mut();
    //     let waiter_id = state.next_waiter_id();
    //     state.register_accel_waiter(waiter_id)?;

    //     Ok(NextScaledAccelFuture {
    //         interface: self,
    //         waiter_id,
    //         done: false,
    //     })
    // }
}

pub struct CommandResponseFuture<'a, R> {
    interface: &'a AsyncInterface<'a>,
    done: bool,
    _marker: core::marker::PhantomData<R>,
}

impl<'a, R> CommandResponseFuture<'a, R> {
    pub fn new(interface: &'a AsyncInterface<'a>) -> Self {
        Self {
            interface,
            done: false,
            _marker: Default::default(),
        }
    }
}

impl<'a, R> Future for CommandResponseFuture<'a, R>
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
                Poll::Pending
            }
            // if let Some(e) = state.command_error.take() {
            //     this.done = true;
            //     return Poll::Ready(Err(e));
            // }
            // if state.pending_command.is_none() {
            //     // This should be impossible to do
            //     panic!("Pending command is none but future is not completed!");
            // }

            // let pending = state.pending_command.as_mut().unwrap();
            // if !pending.complete {
            //     state.register_command_waker(cx.waker());
            //     return Poll::Pending;
            // } else {
            //     this.done = true;
            //     let result = pending.frame;
            //     state.pending_command = None;

            //     return Poll::Ready(Ok(result));
            // }
        })
    }
}

impl<'a, R> Drop for CommandResponseFuture<'a, R> {
    fn drop(&mut self) {
        if self.done {
            return;
        }
        critical_section::with(|cs| {
            let mut state = self.interface.state.borrow_ref_mut(cs);
            //
            state.pending_command.cancel();
        })
    }
}

// pub struct NextScaledAccelFuture<'a, const N_ACCEL_WAITERS: usize> {
//     interface: &'a AsyncInterface<N_ACCEL_WAITERS>,
//     waiter_id: u64,
//     done: bool,
// }

// impl<const N_ACCEL_WAITERS: usize> Future for NextScaledAccelFuture<'_, N_ACCEL_WAITERS> {
//     type Output = ScaledAccel;

//     fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
//         let this = self.get_mut();
//         if this.done {
//             panic!("polled NextScaledAccelFuture after completion");
//         }

//         let mut state = this.interface.state.borrow_mut();
//         if let Some(accel) = state.take_accel_result(this.waiter_id) {
//             this.done = true;
//             return Poll::Ready(accel);
//         }

//         if state.register_accel_waker(this.waiter_id, cx.waker()) {
//             return Poll::Pending;
//         }

//         panic!("scaled accel waiter missing state");
//     }
// }

// impl<const N_ACCEL_WAITERS: usize> Drop for NextScaledAccelFuture<'_, N_ACCEL_WAITERS> {
//     fn drop(&mut self) {
//         if self.done {
//             return;
//         }

//         let mut state = self.interface.state.borrow_mut();
//         state.cancel_accel_waiter(self.waiter_id);
//     }
// }

// fn is_base_response_for_descriptor(msg: &OwnedMessage, expected_descriptor: u8) -> bool {
//     if msg.descriptor_set() != commands::BASE_DESCRIPTOR_SET {
//         return false;
//     }

//     let payload = msg.payload();
//     let mut idx = 0usize;
//     while idx + 2 <= payload.len() {
//         let field_len = payload[idx] as usize;
//         if field_len < 2 || idx + field_len > payload.len() {
//             break;
//         }

//         let descriptor = payload[idx + 1];
//         if descriptor == expected_descriptor {
//             return true;
//         }

//         if descriptor == 0xF1 && field_len >= 4 && payload[idx + 2] == expected_descriptor {
//             return true;
//         }

//         idx += field_len;
//     }

//     false
// }
