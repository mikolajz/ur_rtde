//! Example of usage of RTDEMultiplexedConnection.
//!
//! Sets the Standard Outputs 4-7 to match the Standard Output 0-3. One can toggle the
//! values in pendant and after 5 toggles, the program finishes.

use std::time::Instant;

use ur_rtde::raw_rtde::{GenericRawRTDE, RTDEData};
use ur_rtde::rtde_mux::{
    RTDEMultiplexedConnectionBuilder, RTDEReadVariableSet, RTDEWriteVariableSet,
};

#[derive(Debug)]
struct DigitalOutputVars {
    pub standard_digital_output: u8,
    pub standard_digital_output_mask: u8,
}

impl RTDEWriteVariableSet for DigitalOutputVars {
    fn get_variables() -> Vec<String> {
        vec![
            String::from("standard_digital_output"),
            String::from("standard_digital_output_mask"),
        ]
    }
    fn convert_into_rtde_data(&self) -> Vec<RTDEData> {
        vec![
            RTDEData::Uint8(self.standard_digital_output),
            RTDEData::Uint8(self.standard_digital_output_mask),
        ]
    }
}

#[derive(Debug)]
struct DigitalInputVars {
    pub actual_digital_output_bits: u64,
}

impl RTDEReadVariableSet for DigitalInputVars {
    fn get_variables() -> Vec<String> {
        vec![String::from("actual_digital_output_bits")]
    }
    fn fill_from_rtde_data(data: &[RTDEData]) -> Result<DigitalInputVars, String> {
        if data.len() != 1 {
            return Err(format!("Got {} elements instead of 1", data.len()));
        }

        return Ok(DigitalInputVars {
            actual_digital_output_bits: {
                match data[0] {
                    RTDEData::Uint64(d) => Ok(d),
                    _ => Err(format!("Wrong type for field 0: {:?}", data[0])),
                }
            }?,
        });
    }
}

pub fn run_rtde_echo() {
    const TOGGLE_LIMIT: i32 = 5;

    let rtde = GenericRawRTDE::connect("127.0.0.1:30004".parse().unwrap()).unwrap();
    let mut mux_builder = RTDEMultiplexedConnectionBuilder::new(rtde, 500.0);

    let mut outputs = DigitalOutputVars {
        standard_digital_output: 0,
        standard_digital_output_mask: 0xf0, // Will modify only the 4 higher bits.
    };

    let io_reader_promise = mux_builder
        .add_read_variable_set::<DigitalInputVars>()
        .unwrap();
    let io_writer_promise = mux_builder.add_write_variable_set(&outputs).unwrap();

    let connection = mux_builder.connect().unwrap();
    let io_reader = connection.get_reader(io_reader_promise).unwrap();
    let io_writer = connection.get_writer(io_writer_promise).unwrap();

    println!("Please toggle inputs of the robot to see a change in the outputs");
    let mut prev_packet = None;
    let mut num_toggles = 0;
    let mut num_packets = 0;

    let start = Instant::now();
    loop {
        let (packet_id, data) = io_reader.get_latest_packet(&prev_packet).unwrap();
        prev_packet = Some(packet_id);
        num_packets += 1;

        let low_4_standard_outputs = (data.actual_digital_output_bits & 0xf) as u8;
        let wanted_high_4_outputs = low_4_standard_outputs << 4;
        if wanted_high_4_outputs != outputs.standard_digital_output {
            println!("Applying new I/O value: {low_4_standard_outputs:#x}");
            outputs.standard_digital_output = wanted_high_4_outputs;
            io_writer.send(&outputs).unwrap();

            num_toggles += 1;
            if num_toggles > TOGGLE_LIMIT {
                println!("Reached {TOGGLE_LIMIT} toggles - finishing.");
                break;
            }
        }
    }

    let runtime = start.elapsed().as_secs_f32();
    let pps = num_packets as f32 / runtime;
    println!("Received {num_packets} packets in {runtime:.1} ({pps:.1} packets/sec).");
}

fn main() {
    env_logger::init();

    run_rtde_echo();
}
