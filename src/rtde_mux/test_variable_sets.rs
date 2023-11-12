#[cfg(test)]
pub(crate) mod tests {
    use crate::raw_rtde::RTDEData;
    use crate::rtde_mux::{RTDEReadVariableSet, RTDEWriteVariableSet};

    
    pub(crate) struct WriteVariableSetCB {
        pub standard_digital_output: u8,  // Also called "C" for short.
        pub standard_digital_output_mask: u8,  // Also called "B" for short.
    }

    impl RTDEWriteVariableSet for WriteVariableSetCB {
        fn get_variables() -> Vec<String> { vec![String::from("standard_digital_output"), String::from("standard_digital_output_mask")]}
        fn convert_into_rtde_data(&self) -> Vec<RTDEData> {
            vec![RTDEData::Uint8(self.standard_digital_output), RTDEData::Uint8(self.standard_digital_output_mask)]
        }
    }

    pub(crate) struct WriteVariableSetAB {
        pub standard_analog_output_0: f64,  // Also called "A" for short.
        pub standard_digital_output_mask: u8,  // Also called "B" for short.
    }
    impl RTDEWriteVariableSet for WriteVariableSetAB {
        fn get_variables() -> Vec<String> { vec![String::from("standard_analog_output_0"), String::from("standard_digital_output_mask")]}
        fn convert_into_rtde_data(&self) -> Vec<RTDEData> {
            vec![RTDEData::Double(self.standard_analog_output_0), RTDEData::Uint8(self.standard_digital_output_mask)]
        }
    }

    #[derive(PartialEq, Debug)]
    pub(crate) struct ReadVariableSetXY {
        pub timestamp: f64,  // Also called "X" for short.
        pub robot_mode: i32,  // Also called "Y" for short.
    }
    impl RTDEReadVariableSet for ReadVariableSetXY {
        fn get_variables() -> Vec<String> { vec![String::from("timestamp"), String::from("robot_mode")]}
        fn fill_from_rtde_data(data: &[RTDEData]) -> Result<ReadVariableSetXY, String> {
            if data.len() != 2 {
                return Err(format!("Got {} elements instead of 2", data.len()));
            }

            return Ok(ReadVariableSetXY{
                timestamp: {
                    match data[0] {
                        RTDEData::Double(d) => Ok(d),
                        _ => Err(format!("Wrong type for field 0: {:?}", data[0])),
                    }
                }?,
                robot_mode: {
                    match data[1] {
                        RTDEData::Int32(d) => Ok(d),
                        _ => Err(format!("Wrong type for field 1: {:?}", data[1])),
                    }
                }?,
            })
        }
    }

    #[derive(PartialEq, Debug)]
    pub(crate) struct ReadVariableSetYZ {
        pub robot_mode: i32,  // Also called "Y" for short.
        pub payload: f64,  // Also called "Z" for short.
    }
    impl RTDEReadVariableSet for ReadVariableSetYZ {
        fn get_variables() -> Vec<String> { vec![String::from("robot_mode"), String::from("payload")]}
        fn fill_from_rtde_data(data: &[RTDEData]) -> Result<ReadVariableSetYZ, String> {
            if data.len() != 2 {
                return Err(format!("Got {} elements instead of 2", data.len()));
            }

            return Ok(ReadVariableSetYZ{
                robot_mode: {
                    match data[0] {
                        RTDEData::Int32(d) => Ok(d),
                        _ => Err(format!("Wrong type for field 0: {:?}", data[0])),
                    }
                }?,
                payload: {
                    match data[1] {
                        RTDEData::Double(d) => Ok(d),
                        _ => Err(format!("Wrong type for field 1: {:?}", data[1])),
                    }
                }?,
            })
        }
    }

}