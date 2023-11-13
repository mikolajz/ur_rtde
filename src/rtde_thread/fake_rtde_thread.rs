use std::cell::RefCell;
use std::rc::Rc;

use crate::raw_rtde::RTDEData;
use crate::rtde_thread::{RTDEThreadTrait, Packet, PacketId};

/// Controls the behaviour of a `FakeRTDEThread` and allow to query how its used.
pub struct FakeRTDEThreadControl {
    /// Packets sent by the user of the FakeRTDEThread.
    pub sent_packets: Vec<Packet>,
    /// Whether the user of FakeRTDEThread started the thread.
    pub running: bool,
    // The packet that the next call to `get_latest_packet` will return.
    pub packet_to_receive: Option<Packet>,    
}

/// A fake RTDEThread. Meant for unit tests.
pub struct FakeRTDEThread {
    control: Rc<RefCell<FakeRTDEThreadControl>>,
}

impl FakeRTDEThread {
    pub fn new() -> (FakeRTDEThread, Rc<RefCell<FakeRTDEThreadControl>>) {
        let control = Rc::new(
            RefCell::new(
                FakeRTDEThreadControl{
                    sent_packets: vec![],
                    running: true,
                    packet_to_receive: None,
                }
            )
        );
        (FakeRTDEThread{control: control.clone()}, control)
    }
}

impl RTDEThreadTrait for FakeRTDEThread {
    fn stop(&mut self) -> Result<(), String> {
        if !self.control.borrow().running {
            return Err(String::from("Trying to stop an already stopped thread"));
        }

        self.control.borrow_mut().running = false;
        Ok(())
    }

    fn send(&self, data: &[RTDEData]) -> Result<(), String> {
        if !self.control.borrow().running {
            return Err(String::from("Trying to use a stopped thread."));
        }

        let packet = Packet{
            packet_id: PacketId(0),
            payload: data.to_vec(),
        };

        self.control.borrow_mut().sent_packets.push(packet);
        Ok(())
    }

    fn get_latest_packet(&self, _previous_id: &Option<PacketId>) -> Result<Packet, String> {
        match &self.control.borrow().packet_to_receive {
            Some(packet) => Ok(packet.clone()),
            None => Err(String::from("No packet to receive configured in FakeRTDEThread")),
        }
    }
}