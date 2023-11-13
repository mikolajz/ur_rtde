//! Example of usage of RawRTDE
//!
//! Creates a connection and queries a few values.

use ur_rtde::raw_rtde::{GenericRawRTDE, RTDEData, RawRTDETrait};

fn test_raw_rtde() {
    let mut rtde = GenericRawRTDE::connect("127.0.0.1:30004".parse().unwrap()).unwrap();
    let controller_version = rtde.get_controller_version().unwrap();
    println!("{}", controller_version);
    let inputs_setup = rtde
        .configure_inputs(&["standard_digital_output_mask", "standard_digital_output"])
        .unwrap();
    println!("{:?}", inputs_setup);
    rtde.send_robot_input(
        &inputs_setup,
        &vec![RTDEData::Uint8(0x7), RTDEData::Uint8(0x3)],
    )
    .unwrap();

    let outputs_setup = rtde
        .configure_outputs(125f64, &["actual_TCP_speed", "actual_TCP_pose"])
        .unwrap();
    rtde.start_robot_to_host_transmition().unwrap();

    let output_packet = rtde.wait_for_robot_output(&outputs_setup).unwrap();
    println!("Received a packet");
    for data in output_packet {
        println!("Data: {:?}", &data)
    }
}

fn main() {
    env_logger::init();

    test_raw_rtde();
}
