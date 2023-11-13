use std::fmt::{Display, Formatter};
use std::io::Cursor;
use std::mem::{discriminant, Discriminant};

use byteorder::{BigEndian, ReadBytesExt, WriteBytesExt};

pub struct ControllerVersion {
    pub major: i32,
    pub minor: i32,
    pub bugfix: i32,
    pub build: i32,
}

impl Display for ControllerVersion {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(
            f,
            "{}.{}.{}.{}",
            self.major, self.minor, self.bugfix, self.build
        )
    }
}

/// A single value in a robot input or output data packet.
#[derive(Debug, Clone, PartialEq)]
pub enum RTDEData {
    Int32(i32),
    Uint32(u32),
    Vector6D([f64; 6]),
    Vector3D([f64; 3]),
    Vector6Int32([i32; 6]),
    Vector3Int32([i32; 3]),
    Double(f64),
    Uint64(u64),
    Uint8(u8),
}

impl RTDEData {
    /// Creates a zero value for a given type name.
    ///
    /// The type name corresponds to the one used in RTDE protocol.
    pub fn default_for_type_name(type_name: &str) -> Result<RTDEData, String> {
        match type_name {
            "INT32" => Ok(RTDEData::Int32(0)),
            "UINT32" => Ok(RTDEData::Uint32(0)),
            "VECTOR6D" => Ok(RTDEData::Vector6D([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])),
            "VECTOR3D" => Ok(RTDEData::Vector3D([0.0, 0.0, 0.0])),
            "VECTOR6Int32" => Ok(RTDEData::Vector6Int32([0, 0, 0, 0, 0, 0])),
            "VECTOR3Int32" => Ok(RTDEData::Vector3Int32([0, 0, 0])),
            "DOUBLE" => Ok(RTDEData::Double(0.0)),
            "UINT64" => Ok(RTDEData::Uint64(0)),
            "UINT8" => Ok(RTDEData::Uint8(0)),
            _ => Err(format!("Unknown type name {}", type_name)),
        }
    }

    /// Serialize the value as it's transmitted on the wire in RTDE.
    pub fn serialize(&self, result: &mut Vec<u8>) {
        match self {
            RTDEData::Int32(d) => result.write_i32::<BigEndian>(*d).unwrap(),
            RTDEData::Uint32(d) => result.write_u32::<BigEndian>(*d).unwrap(),
            RTDEData::Vector6D(array) => {
                for d in array.iter() {
                    result.write_f64::<BigEndian>(*d).unwrap();
                }
            }
            RTDEData::Vector3D(array) => {
                for d in array.iter() {
                    result.write_f64::<BigEndian>(*d).unwrap();
                }
            }
            RTDEData::Vector6Int32(array) => {
                for d in array.iter() {
                    result.write_i32::<BigEndian>(*d).unwrap();
                }
            }
            RTDEData::Vector3Int32(array) => {
                for d in array.iter() {
                    result.write_i32::<BigEndian>(*d).unwrap();
                }
            }
            RTDEData::Double(d) => result.write_f64::<BigEndian>(*d).unwrap(),
            RTDEData::Uint64(d) => result.write_u64::<BigEndian>(*d).unwrap(),
            RTDEData::Uint8(d) => result.write_u8(*d).unwrap(),
        }
    }

    /// Deserialize the value from the format it's transmited on the wire in RTDE.
    pub fn parse(
        input: &mut Cursor<&[u8]>,
        data_type: Discriminant<RTDEData>,
    ) -> Result<RTDEData, String> {
        // Can this be replaced with `match`?
        if data_type == discriminant(&RTDEData::Int32(0)) {
            Ok(RTDEData::Int32(
                input
                    .read_i32::<BigEndian>()
                    .map_err(|_e| "Data packet too short")?,
            ))
        } else if data_type == discriminant(&RTDEData::Uint32(0)) {
            Ok(RTDEData::Uint32(
                input
                    .read_u32::<BigEndian>()
                    .map_err(|_e| "Data packet too short")?,
            ))
        } else if data_type == discriminant(&RTDEData::Vector6D([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])) {
            let mut array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            for item in &mut array {
                *item = input
                    .read_f64::<BigEndian>()
                    .map_err(|_e| "Data packet too short")?;
            }
            Ok(RTDEData::Vector6D(array))
        } else if data_type == discriminant(&RTDEData::Vector3D([0.0, 0.0, 0.0])) {
            let mut array = [0.0, 0.0, 0.0];
            for item in &mut array {
                *item = input
                    .read_f64::<BigEndian>()
                    .map_err(|_e| "Data packet too short")?;
            }
            Ok(RTDEData::Vector3D(array))
        } else if data_type == discriminant(&RTDEData::Vector6Int32([0, 0, 0, 0, 0, 0])) {
            let mut array = [0, 0, 0, 0, 0, 0];
            for item in &mut array {
                *item = input
                    .read_i32::<BigEndian>()
                    .map_err(|_e| "Data packet too short")?;
            }
            Ok(RTDEData::Vector6Int32(array))
        } else if data_type == discriminant(&RTDEData::Vector3Int32([0, 0, 0])) {
            let mut array = [0, 0, 0];
            for item in &mut array {
                *item = input
                    .read_i32::<BigEndian>()
                    .map_err(|_e| "Data packet too short")?;
            }
            Ok(RTDEData::Vector3Int32(array))
        } else if data_type == discriminant(&RTDEData::Double(0.0)) {
            Ok(RTDEData::Double(
                input
                    .read_f64::<BigEndian>()
                    .map_err(|_e| "Data packet too short")?,
            ))
        } else if data_type == discriminant(&RTDEData::Uint64(0)) {
            Ok(RTDEData::Uint64(
                input
                    .read_u64::<BigEndian>()
                    .map_err(|_e| "Data packet too short")?,
            ))
        } else if data_type == discriminant(&RTDEData::Uint8(0)) {
            Ok(RTDEData::Uint8(
                input.read_u8().map_err(|_e| "Data packet too short")?,
            ))
        } else {
            panic!("Unhandled discriminant");
        }
    }
}

/// Description of input or output setup as returned by the robot in RTDE.
#[derive(Clone, PartialEq, Debug)]
pub struct IoSetup {
    /// Recipe id.
    ///
    /// RTDE allows to set up multiple set of variables transmitted from robot to host,
    /// with different frequencies.
    pub recipe_id: u8,
    /// Types of variables.
    ///
    /// For the names of variables provided by the host, the robot replis with their types.
    pub types: Vec<Discriminant<RTDEData>>,
}
