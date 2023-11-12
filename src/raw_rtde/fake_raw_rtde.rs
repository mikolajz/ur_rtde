use std::mem::{discriminant, Discriminant};
use std::sync::{Arc, Mutex, Condvar};
use std::time::Duration;

use phf::phf_map;

use crate::raw_rtde::{IoSetup, RawRTDETrait, RTDEData};


// Should be good enough for unit tests. If not, we can make it a parameter.
const RECEIVE_DATA_TIMEOUT: Duration = Duration::from_millis(1000);

static FAKE_INPUTS: phf::Map<&'static str, RTDEData> = phf_map! {
    "standard_digital_output_mask" => RTDEData::Uint8(0),
    "standard_digital_output" => RTDEData::Uint8(0),
    "standard_analog_output_0" => RTDEData::Double(0.0),
};

static FAKE_OUTPUTS: phf::Map<&'static str, RTDEData> = phf_map! {
    "timestamp" => RTDEData::Double(0.0),
    "robot_mode" => RTDEData::Int32(0),
    "payload" => RTDEData::Double(0.0),
};

#[derive(PartialEq, Clone, Debug)]
pub struct FakeIoConfig {
    pub vars: Vec<String>,
    pub io_setup: IoSetup,
}

pub enum QueueItemMode {
    OneTime,
    Repeat,
}
struct QueuedOutput {
    mode: QueueItemMode,
    recipe_id: u8,
    data: Vec<RTDEData>,
}

/// Controls the behaviour of a `FakeRawRTDE`.
/// 
/// The FakeRawRTDE is usually passed to the code under test, while this object
/// can stay in the tester code and control how the former behaves.
pub struct FakeRawRTDEControl {
    /// Whether the user of FakeRawRTDE started the robot to host transmition.
    pub transmission_started: bool,
    /// The input variables configured by the user of FakeRawRTDE.
    pub input_io_setup: Option<FakeIoConfig>, 
    /// The output variables configured by the user of FakeRawRTDE.
    pub output_io_setups: Vec<FakeIoConfig>,
    /// Robot input packets received from the user of FakeRawRTDE.
    pub sent_input_packets: Vec<Vec<RTDEData>>,
    queued_output_packets: Vec<QueuedOutput>,
    wait_for_output_interrupt_request: bool,
    new_output: Arc<Condvar>,  // Using Rc to keep a reference when releasing the mutex holding this.
}

impl FakeRawRTDEControl {

    /// Make the FakeRawRTDE repeatedly return the given robot output packet.
    pub fn queue_repeating_output(&mut self, recipe_id: u8, data: &[RTDEData]) -> () {
        self.queue_output_worker(recipe_id, data, QueueItemMode::Repeat);
    }

    /// Make the FakeRawRTDE one time return the given robot output packet.
    /// 
    /// If it gets asked for another output packet, it will require another call to
    /// `queue_output` or `queue_repeating_output` to proceed.
    pub fn queue_output(&mut self, recipe_id: u8, data: &[RTDEData]) -> () {
        self.queue_output_worker(recipe_id, data, QueueItemMode::OneTime);
    }

    fn queue_output_worker(&mut self, recipe_id: u8, data: &[RTDEData], queue_item_mode: QueueItemMode) -> () {
        // Right now this check is disabled, as in some tests we need to queue a packet before the outputs are configured.
        //assert!((0..self.output_io_setups.len()).contains(&recipe_id.into()), "Recipe {} out of range", recipe_id);

        self.queued_output_packets.push(QueuedOutput{
            mode: queue_item_mode,
            recipe_id,
            data: data.to_vec(),
        });
        self.new_output.notify_all()
    }

}

/// Fake implementation of RawRTDE. Meant for unit tests.
pub struct FakeRawRTDE {
    rtde_control: Arc<Mutex<FakeRawRTDEControl>>,
}

impl FakeRawRTDE {

    /// Returns a new FakeRawRTDE and a control object to control the former. 
    pub fn new_fake_and_control() -> (FakeRawRTDE, Arc<Mutex<FakeRawRTDEControl>>){
        let control = Arc::new(
            Mutex::new(
                FakeRawRTDEControl{
                    transmission_started: false,
                    input_io_setup: None,
                    output_io_setups: vec![],
                    sent_input_packets: vec![],
                    queued_output_packets: vec![],
                    wait_for_output_interrupt_request: false,
                    new_output: Arc::new(Condvar::new()),
                }
            )
        );

        let fake = FakeRawRTDE{
            rtde_control: control.clone(),
        };

        (fake, control)
    }

    fn map_type(variable: &str, is_input: bool) -> Discriminant<RTDEData> {
        let fake_vars = if is_input { &FAKE_INPUTS} else { &FAKE_OUTPUTS};

        discriminant(fake_vars.get(variable).unwrap_or_else(
            || panic!("Variable {variable} not supported - add to FAKE_VARS or fix typo")),
        )
    }

    fn map_types<U: 'static + AsRef<str>>(variables: &[U], is_input: bool) -> Vec<Discriminant<RTDEData>> {
        variables.iter().map(|x| Self::map_type(x.as_ref(), is_input)).collect()
    }

}

impl RawRTDETrait for FakeRawRTDE {
    fn start_robot_to_host_transmition(&mut self) -> Result<(), String> {
        let mut control = self.rtde_control.lock().unwrap();

        assert!(!control.transmission_started);
        control.transmission_started = true;

        Ok(())
    }

    fn pause_robot_to_host_transmition(&mut self) -> Result<(), String> {
        let mut control = self.rtde_control.lock().unwrap();

        assert!(control.transmission_started);
        control.transmission_started = false;

        Ok(())
    }

    fn configure_inputs<U: 'static + AsRef<str>>(&mut self, variables: &[U]) -> Result<IoSetup, String> {
        let types = Self::map_types(variables, true);

        let result = IoSetup{
            recipe_id: 0,
            types,
        };

        let mut control = self.rtde_control.lock().unwrap();

        assert!(!control.transmission_started);  // Is this enforced by UR?
        assert!(control.input_io_setup.is_none());
        control.input_io_setup = Some(FakeIoConfig{
            vars: variables.iter().map(|s| String::from(s.as_ref())).collect(),
            io_setup: result.clone(),
        });

        Ok(result)
    }

    fn configure_outputs<U: 'static + AsRef<str>>(&mut self, _frequency: f64, variables: &[U]) -> Result<IoSetup, String> {
        let types = Self::map_types(variables, false);

        let mut control = self.rtde_control.lock().unwrap();

        let result = IoSetup{
            recipe_id: control.output_io_setups.len().try_into().unwrap(),
            types,
        };

        assert!(!control.transmission_started);  // Is this enforced by UR?
        control.output_io_setups.push(FakeIoConfig{
            vars: variables.iter().map(|s| String::from(s.as_ref())).collect(),
            io_setup: result.clone(),
        });

        Ok(result)
    }

    fn send_robot_input(&mut self, io_setup: &IoSetup, input_data: &[RTDEData]) -> Result<(), String> {
        let mut control = self.rtde_control.lock().unwrap();
        assert!(io_setup == &control.input_io_setup.as_ref().expect("Attempt to send inputs before configuring them.").io_setup);

        control.sent_input_packets.push(input_data.to_vec());

        Ok(())
    }

    fn wait_for_robot_output(&mut self, io_setup: &IoSetup) -> Result<Option<Vec<RTDEData>>, String> {
        let mut control = self.rtde_control.lock().unwrap();

        loop {
            if control.wait_for_output_interrupt_request {
                control.wait_for_output_interrupt_request = false;
                return Ok(None);
            }

            if let Some((index, item)) = control.queued_output_packets.iter().enumerate().filter(|(_, item)| item.recipe_id == io_setup.recipe_id).nth(0) {
                let data = match item.mode {
                    QueueItemMode::OneTime => control.queued_output_packets.remove(index).data,
                    QueueItemMode::Repeat => control.queued_output_packets[index].data.clone(),
                };
                return Ok(Some(data));
            }

            let new_output_condvar = control.new_output.clone();
            let timeout_result;
            (control, timeout_result) = new_output_condvar.wait_timeout(control, RECEIVE_DATA_TIMEOUT).unwrap();

            if timeout_result.timed_out() {
                panic!("Timed out waiting for result");
            }
        }
    }

    fn get_wait_for_robot_output_interrupter(&self) -> Box<dyn Fn() -> Result<(), String> + Send + 'static> {
        let control_arc = self.rtde_control.clone();
        Box::new(move || {
            let mut control = control_arc.lock().unwrap();
            control.wait_for_output_interrupt_request = true;
            control.new_output.notify_all();
            Ok(())
        })
    }
}

#[cfg(test)]
mod tests {

    use std::thread;
    use std::time::Duration;

    use super::*;

    #[ctor::ctor]
    fn init() {
        let _ = env_logger::builder().is_test(true).try_init();
    }

    #[test]
    fn handles_start_stop() {
        let (mut fake, control) = FakeRawRTDE::new_fake_and_control();

        assert!(!control.lock().unwrap().transmission_started);

        fake.start_robot_to_host_transmition().unwrap();

        assert!(control.lock().unwrap().transmission_started);

        fake.pause_robot_to_host_transmition().unwrap();

        assert!(!control.lock().unwrap().transmission_started);
    }

    #[test]
    #[should_panic(expected="transmission_started")]
    fn cant_start_twice() {
        let (mut fake, _control) = FakeRawRTDE::new_fake_and_control();
        fake.start_robot_to_host_transmition().unwrap();
        fake.start_robot_to_host_transmition().unwrap();
    }

    #[test]
    #[should_panic(expected="transmission_started")]
    fn cant_stop_twice() {
        let (mut fake, _control) = FakeRawRTDE::new_fake_and_control();
        fake.start_robot_to_host_transmition().unwrap();
        fake.start_robot_to_host_transmition().unwrap();
    }

    #[test]
    fn stores_inputs_and_outputs() {
        let (mut fake, control) = FakeRawRTDE::new_fake_and_control();

        let inputs_io_setup = fake.configure_inputs(&["standard_analog_output_0", "standard_digital_output"]).unwrap();
        let outputs_io_setup_1 = fake.configure_outputs(500.0, &["timestamp", "robot_mode"]).unwrap();
        let outputs_io_setup_2 = fake.configure_outputs(125.0, &["payload"]).unwrap();

        assert_eq!(inputs_io_setup, IoSetup{
            recipe_id: 0,
            types: vec![
                discriminant(&RTDEData::Double(0.0)),
                discriminant(&RTDEData::Uint8(0)),
            ]
        });
        assert_eq!(outputs_io_setup_1, IoSetup{
            recipe_id: 0,
            types: vec![
                discriminant(&RTDEData::Double(0.0)),
                discriminant(&RTDEData::Int32(0)),
            ]
        });
        assert_eq!(outputs_io_setup_2, IoSetup{
            recipe_id: 1,
            types: vec![
                discriminant(&RTDEData::Double(0.0)),
            ]
        });

        assert_eq!(control.lock().unwrap().input_io_setup, Some(FakeIoConfig{
            vars: vec![String::from("standard_analog_output_0"), String::from("standard_digital_output")],
            io_setup: inputs_io_setup,
        }));
        assert_eq!(control.lock().unwrap().output_io_setups, vec![
            FakeIoConfig{
                vars: vec![String::from("timestamp"), String::from("robot_mode")],
                io_setup: outputs_io_setup_1,
            },
            FakeIoConfig{
                vars: vec![String::from("payload")],
                io_setup: outputs_io_setup_2,
            },
        ]);
    }

    #[test]
    fn sends_outputs_queued_early() {
        let (mut fake, control) = FakeRawRTDE::new_fake_and_control();
        let data_packet_0 = [
            RTDEData::Double(1.0),
            RTDEData::Int32(3),
        ];
        let data_packet_1 = [
            RTDEData::Double(2.0),
            RTDEData::Int32(4),
        ];

        let outputs_io_setup = fake.configure_outputs(500.0, &["timestamp", "robot_mode"]).unwrap();

        control.lock().unwrap().queue_output(outputs_io_setup.recipe_id, &data_packet_0);
        control.lock().unwrap().queue_output(outputs_io_setup.recipe_id, &data_packet_1);

        let received_packet_0 = fake.wait_for_robot_output(&outputs_io_setup).unwrap();
        let received_packet_1 = fake.wait_for_robot_output(&outputs_io_setup).unwrap();

        assert_eq!(outputs_io_setup, IoSetup{
            recipe_id: 0,
            types: vec![
                discriminant(&RTDEData::Double(0.0)),
                discriminant(&RTDEData::Int32(0)),
            ]
        });
        assert_eq!(received_packet_0.unwrap(), data_packet_0);
        assert_eq!(received_packet_1.unwrap(), data_packet_1);
    }

    #[test]
    fn sends_outputs_queued_late() {
        let (mut fake, control) = FakeRawRTDE::new_fake_and_control();
        let data_packet_0 = [
            RTDEData::Double(1.0),
            RTDEData::Int32(3),
        ];
        let data_packet_1 = [
            RTDEData::Double(2.0),
            RTDEData::Int32(4),
        ];

        let outputs_io_setup = fake.configure_outputs(500.0, &["timestamp", "robot_mode"]).unwrap();

        let join_handle = {
            let control_other_ref = control.clone();
            let data_packet_0_copy = data_packet_0.clone();
            let data_packet_1_copy = data_packet_1.clone();
            let outputs_io_setup_copy = outputs_io_setup.clone();
            thread::spawn(move || {
                thread::sleep(Duration::from_millis(100));
                control_other_ref.lock().unwrap().queue_output(outputs_io_setup_copy.recipe_id, &data_packet_0_copy);
                control_other_ref.lock().unwrap().queue_output(outputs_io_setup_copy.recipe_id, &data_packet_1_copy);
            })
        };

        let received_packet_0 = fake.wait_for_robot_output(&outputs_io_setup).unwrap();
        let received_packet_1 = fake.wait_for_robot_output(&outputs_io_setup).unwrap();

        assert_eq!(outputs_io_setup, IoSetup{
            recipe_id: 0,
            types: vec![
                discriminant(&RTDEData::Double(0.0)),
                discriminant(&RTDEData::Int32(0)),
            ]
        });
        assert_eq!(received_packet_0.unwrap(), data_packet_0);
        assert_eq!(received_packet_1.unwrap(), data_packet_1);

        join_handle.join().unwrap();
    }

    #[test]
    fn can_interrupt_output_wait() {
        let (mut fake, _control) = FakeRawRTDE::new_fake_and_control();

        let outputs_io_setup = fake.configure_outputs(500.0, &["timestamp", "robot_mode"]).unwrap();

        let interrupter = fake.get_wait_for_robot_output_interrupter();

        let join_handle = thread::spawn(move || {
            thread::sleep(Duration::from_millis(20));
            interrupter().unwrap();
        });

        let opt_received_packet = fake.wait_for_robot_output(&outputs_io_setup).unwrap();

        assert_eq!(opt_received_packet, None);

        join_handle.join().unwrap();
    }

    #[test]
    fn accepts_inputs() {
        let (mut fake, control) = FakeRawRTDE::new_fake_and_control();
        let data_packet_0 = [
            RTDEData::Double(1.0),
            RTDEData::Uint8(42),
        ];
        let data_packet_1 = [
            RTDEData::Double(2.0),
            RTDEData::Uint8(137),
        ];

        let inputs_io_setup = fake.configure_inputs(&["standard_analog_output_0", "standard_digital_output"]).unwrap();

        fake.send_robot_input(&inputs_io_setup, &data_packet_0).unwrap();
        fake.send_robot_input(&inputs_io_setup, &data_packet_1).unwrap();

        assert_eq!(inputs_io_setup, IoSetup{
            recipe_id: 0,
            types: vec![
                discriminant(&RTDEData::Double(0.0)),
                discriminant(&RTDEData::Uint8(0)),
            ]
        });
        assert_eq!(control.lock().unwrap().sent_input_packets, &[data_packet_0, data_packet_1]);
    }

}
