use crate::raw_rtde::RTDEData;

/// ID of a specific packet received from the robot.
///
/// Used to make sure we get the next packet and not the same packet again.
#[derive(PartialEq, Clone, Copy, Debug)]
pub struct PacketId(pub i32);

/// A packet received from the robot.
#[derive(Clone, Debug)]
pub struct Packet {
    /// The ID of the packet.
    pub packet_id: PacketId,
    /// Values of RTDE variables in the packet.
    ///
    /// To names of the variables are the ones passed to RTDEThread when establishing
    /// the connection.
    pub payload: Vec<RTDEData>,
}
