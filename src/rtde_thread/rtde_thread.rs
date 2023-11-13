use std::sync::{Arc, Mutex, Condvar};
use std::sync::mpsc::{channel, Sender, Receiver};
use std::marker::PhantomData;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use crate::raw_rtde::{RTDEData, RawRTDETrait, IoSetup};
use crate::rtde_thread::{Packet, PacketId};


const RECEIVE_TIMEOUT: Duration = Duration::from_millis(200);
const SEND_TIMEOUT: Duration = Duration::from_millis(500);


struct WriteRequest {
    data: Vec<RTDEData>,
    result_sender: Sender<Result<(), String>>,
}

struct SharedState {
    stop_request: bool,
    latest_packet: Option<Packet>,  // None if no packet was received yet.
}

/// A thread handling an RTDE connection to the robot.
/// 
/// It continuously listens to new RTDE packets, allows to send a packet to the
/// robot, query for the latest packet received or wait for a new packet to arrive. 
pub struct RTDEThread<T>
    where T: 'static + RawRTDETrait + Send
{
    shared_state: Arc<Mutex<SharedState>>,
    condition: Arc<Condvar>,
    write_sender: Sender<WriteRequest>,
    join_handle: Option<JoinHandle<Result<(), String>>>,  // Is None when the transmition is stopped.
    wait_for_output_interrupter: Box<dyn Fn() -> Result<(), String> + Send + 'static>,
    phantom: PhantomData<T>,  // In case we need T in the future.
}

/// Methods used by RTDEMultiplexed connection, common to RTDEThread an RawRTDEThread.
pub trait RTDEThreadTrait
{
    /// Stop the robot to host transmition.
    fn stop(&mut self) -> Result<(), String>;

    /// Send a data packet to the robot.
    /// 
    /// The values of variables needs to correspond to the names passed when starting
    /// the connection.
    fn send(&self, data: &[RTDEData]) -> Result<(), String>;

    /// Returns the latest received packet or one later than `newer_than` (if set).
    /// 
    /// When called with `newer_than==None`, this function is usually immediate (the only exception is just
    /// after starting the transmition, when no packet arrived yet). When `newer_than` is set, it may need to
    /// wait for a new packet to arrive.
    fn get_latest_packet(&self, newer_than: &Option<PacketId>) -> Result<Packet, String>;
}

impl<T> RTDEThread<T>
    where T: 'static + RawRTDETrait + Send
{
    /// Starts an RTDE thread with a given set of robot input and robot output variables.
    pub fn start(
        mut raw_rtde: T,
        input_variables: &[String],
        output_variables: &[String],
        read_frequency: f64,
    ) -> Result<RTDEThread<T>, String>
    {
        let output_io_setup = raw_rtde.configure_outputs(read_frequency, output_variables)?;
        let input_io_setup = raw_rtde.configure_inputs(input_variables)?;

        let wait_for_output_interrupter = raw_rtde.get_wait_for_robot_output_interrupter();

        let (write_tx, write_rx) = channel();

        let shared_state_lock = Arc::new(
            Mutex::new(
                SharedState{
                    stop_request: false,
                    latest_packet: None,
                }
            )
        );

        let condition = Arc::new(Condvar::new());

        let join_handle = Self::start_thread(
            raw_rtde,
            input_io_setup,
            output_io_setup,
            shared_state_lock.clone(),
            condition.clone(),
            write_rx,
        );

        Ok(RTDEThread{
            shared_state: shared_state_lock,
            condition,
            write_sender: write_tx,
            join_handle: Some(join_handle),
            wait_for_output_interrupter,
            phantom: PhantomData{},
        })
    }

    fn start_thread(
        mut raw_rtde: T,
        input_io_setup: IoSetup,
        output_io_setup: IoSetup,
        shared_state_lock: Arc<Mutex<SharedState>>,
        condition: Arc<Condvar>,
        write_receiver: Receiver<WriteRequest>,
    ) -> JoinHandle<Result<(), String>> {
        let thread_func = move || {
            let mut packet_id = 0;

            raw_rtde.start_robot_to_host_transmition()?;

            loop {
                // TODO: Handle timeout errors

                // Read a packet.
                let opt_robot_outputs = raw_rtde.wait_for_robot_output(&output_io_setup)?;

                {
                    let mut shared_state = shared_state_lock.lock().unwrap();

                    if shared_state.stop_request {
                        break;
                    }

                    if let Some(robot_outputs) = opt_robot_outputs {
                        shared_state.latest_packet = Some(
                            Packet {
                                packet_id: PacketId(packet_id),
                                payload: robot_outputs
                            }
                        );
                        packet_id += 1;

                        condition.notify_all()
                    }
                }

                // Write requested packets.
                for write_request in write_receiver.try_iter() {
                    write_request.result_sender.send(
                        raw_rtde.send_robot_input(&input_io_setup, &write_request.data)
                    ).map_err(|e| format!("Error sending reply: {e}"))?;
                }
            }

            raw_rtde.pause_robot_to_host_transmition()?;

            Ok(())
        };

        thread::spawn(thread_func)
    }

    fn good_latest_packet(previous_id: &Option<PacketId>, shared_state: &SharedState) -> bool {
        // A packet is good if its ID is different than `previous_id` (meaning it's newer) and... it exists :) .
        match (&shared_state.latest_packet, &previous_id) {
            (Some(packet), Some(previous_id)) if packet.packet_id == *previous_id => false,
            (None, _) => false,
            _ => true,
        }
    }

}

impl<T> RTDEThreadTrait for RTDEThread<T>
    where T: 'static + RawRTDETrait + Send
{
    fn stop(&mut self) -> Result<(), String> {
        let taken_join_handle = self.join_handle.take();

        if let Some(join_handle) = taken_join_handle {
            {
                let mut shared_state = self.shared_state.lock().unwrap();
                shared_state.stop_request = true;
            }
            (self.wait_for_output_interrupter)()?;  // Make the thread process the request faster.

            let _ = join_handle.join().map_err(|e| format!("Got error from receiver thread: {:?}", e))?;
    
            Ok(())
        } else {
            // Thread already stopped.
            Ok(())
        }
    }

    fn send(&self, data: &[RTDEData]) -> Result<(), String>
    {
        let (result_tx, result_rx) = channel();
        self.write_sender.send(WriteRequest{
            data: Vec::from(data),
            result_sender: result_tx,
        }).map_err(|e| format!("RTDE thead died {:?}", e))?;

        (self.wait_for_output_interrupter)()?;  // Make the thread process the request faster.

        match result_rx.recv_timeout(SEND_TIMEOUT) {
            Ok(write_result) => write_result,
            Err(e) => Err(format!("Could not communicate with RTDE thread: {e}")),
        }

    }

    fn get_latest_packet(&self, previous_id: &Option<PacketId>) -> Result<Packet, String> {
        let shared_state = self.shared_state.lock().unwrap();

        let (shared_state, timeout) = self.condition.wait_timeout_while(
            shared_state,
            RECEIVE_TIMEOUT,
            |shared_state| -> bool { !Self::good_latest_packet(previous_id, shared_state) },
        ).unwrap();

        if timeout.timed_out() {
            Err(String::from("Timeout waiting for RTDE packet."))
        } else {
            Ok(shared_state.latest_packet.as_ref().unwrap().clone())  // unwrap is safe due to `good_last_packet`. 
        }
    }

}

impl<T> Drop for RTDEThread<T>
    where T: RawRTDETrait + Send
{
    fn drop(&mut self) {
        if let Err(e) = self.stop() {
            // This is dangerous, as if we drop because of a frame unwinding due to a panic, this will cause a message
            // "panic during handling a panic" with no further details. Should we do it differently?
            panic!("{}", e);
        }
    }
}

#[cfg(test)]
mod tests
{
    use super::*;
    use crate::raw_rtde::FakeRawRTDE;

    #[ctor::ctor]
    fn init()
    {
        let _ = env_logger::builder().is_test(true).try_init();
    }

    #[test]
    fn initializes_transmition_properly() {
        let (fake, control) = FakeRawRTDE::new_fake_and_control();

        let inputs = [String::from("standard_digital_output"), String::from("standard_analog_output_0")];
        let outputs = [String::from("timestamp"), String::from("robot_mode")];

        let mut thread = RTDEThread::start(fake, &inputs, &outputs, 125.0).unwrap();

        thread.stop().unwrap();

        assert_eq!(control.lock().unwrap().input_io_setup.as_ref().unwrap().vars, inputs);
        assert_eq!(control.lock().unwrap().output_io_setups.len(), 1);
        assert_eq!(control.lock().unwrap().output_io_setups[0].vars, outputs);
    }

    #[test]
    fn sends_robot_inputs() {
        let (fake, control) = FakeRawRTDE::new_fake_and_control();

        let inputs = [String::from("standard_digital_output"), String::from("standard_analog_output_0")];
        let outputs = [];

        let input_packet_0 = [RTDEData::Uint8(0), RTDEData::Double(42.0)];
        let input_packet_1 = [RTDEData::Uint8(1), RTDEData::Double(137.0)];

        let mut thread = RTDEThread::start(fake, &inputs, &outputs, 125.0).unwrap();

        thread.send(&input_packet_0).unwrap();
        thread.send(&input_packet_1).unwrap();

        assert_eq!(control.lock().unwrap().sent_input_packets, vec![input_packet_0, input_packet_1]);
        thread.stop().unwrap();
    }


    #[test]
    fn receives_robot_outputs() {
        let (fake, control) = FakeRawRTDE::new_fake_and_control();

        let inputs = [];
        let outputs = [String::from("timestamp")];

        let output_packet_0 = [RTDEData::Double(0.0)];
        let output_packet_1 = [RTDEData::Double(42.0)];
        let output_packet_2 = [RTDEData::Double(137.0)];

        let mut thread = RTDEThread::start(fake, &inputs, &outputs, 125.0).unwrap();

        control.lock().unwrap().queue_output(0, &output_packet_0);
        let read_packet_0 = thread.get_latest_packet(&None).unwrap();

        assert_eq!(read_packet_0.payload, output_packet_0);

        control.lock().unwrap().queue_output(0, &output_packet_1);
        let read_packet_1 = thread.get_latest_packet(&Some(read_packet_0.packet_id)).unwrap();

        assert_eq!(read_packet_1.payload, output_packet_1);

        let output_packet_2_copy = output_packet_2.clone();
        let control_copy = control.clone();
        let join_handle = thread::spawn(move || {
            thread::sleep(Duration::from_millis(50));
            control_copy.lock().unwrap().queue_output(0, &output_packet_2_copy);
        });
        let read_packet_2 = thread.get_latest_packet(&Some(read_packet_1.packet_id)).unwrap();
        join_handle.join().unwrap();

        assert_eq!(read_packet_2.payload, output_packet_2);

        control.lock().unwrap().queue_repeating_output(0, &[
            RTDEData::Double(0.0),
            RTDEData::Int32(0),
        ]);  // Make the thread progress. Right now it's required to make stop() work.
        thread.stop().unwrap();
    }

}
