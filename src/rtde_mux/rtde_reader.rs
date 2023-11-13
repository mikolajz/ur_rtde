use std::collections::HashMap;
use std::marker::PhantomData;
use std::sync::Arc;

use crate::raw_rtde::RTDEData;
use crate::rtde_mux::RTDEReadVariableSet;
use crate::rtde_thread::{PacketId, RTDEThreadTrait};

/// A reader (i.e. robot to host transmition) of a specific set of RTDE variables.
pub struct RTDEReader<U: RTDEReadVariableSet> {
    rtde_thread: Arc<dyn RTDEThreadTrait>,
    var_indices: Vec<usize>, // var_indices[i] is the index in thread output packet of the i-th variable in U.
    phantom: PhantomData<U>,
}

impl<U: RTDEReadVariableSet> RTDEReader<U> {
    pub(crate) fn new(
        rtde_thread: &Arc<dyn RTDEThreadTrait>,
        var_names_to_index_in_thread_output: &HashMap<String, usize>,
    ) -> Result<Self, String> {
        let var_names = U::get_variables();
        let var_indices: Vec<usize> = var_names
            .iter()
            .map(|name| {
                var_names_to_index_in_thread_output
                    .get(name)
                    .copied()
                    .ok_or(format!("Variable {} not in packet", name))
            })
            .collect::<Result<_, _>>()?;

        Ok(RTDEReader {
            rtde_thread: rtde_thread.clone(),
            var_indices,
            phantom: PhantomData {},
        })
    }

    /// Returns the latest received packet or one later than `newer_than` (if set).
    ///
    /// When called with `newer_than==None`, this function is usually immediate (the only exception is just
    /// after starting the transmition, when no packet arrived yet). When `newer_than` is set, it may need to
    /// wait for a new packet to arrive.
    pub fn get_latest_packet(
        &self,
        newer_than: &Option<PacketId>,
    ) -> Result<(PacketId, U), String> {
        let packet = self.rtde_thread.get_latest_packet(newer_than)?;
        let variable_set_data: Vec<RTDEData> = self
            .var_indices
            .iter()
            .map(|i| packet.payload[*i].clone())
            .collect();
        let variable_set = U::fill_from_rtde_data(&variable_set_data)?;

        Ok((packet.packet_id, variable_set))
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::rtde_mux::test_variable_sets::tests::{ReadVariableSetXY, ReadVariableSetYZ};
    use crate::rtde_thread::{FakeRTDEThread, Packet};

    #[test]
    fn test_read_from_two_readers() {
        let (fake_thread, fake_thread_control) = FakeRTDEThread::new();
        let fake_thread_ptr: Arc<dyn RTDEThreadTrait> = Arc::new(fake_thread);

        let thread_variables_to_index_map: HashMap<String, usize> = HashMap::from([
            (String::from("payload"), 0),
            (String::from("robot_mode"), 1),
            (String::from("timestamp"), 2),
        ]);
        fake_thread_control.borrow_mut().packet_to_receive = Some(Packet {
            packet_id: PacketId(33),
            payload: vec![
                RTDEData::Double(7.5),  // payload
                RTDEData::Int32(7),     // robot_mode
                RTDEData::Double(1.25), // timestamp
            ],
        });

        let reader1 =
            RTDEReader::<ReadVariableSetXY>::new(&fake_thread_ptr, &thread_variables_to_index_map)
                .unwrap();
        let reader2 =
            RTDEReader::<ReadVariableSetYZ>::new(&fake_thread_ptr, &thread_variables_to_index_map)
                .unwrap();

        let (packet_id, packet) = reader1.get_latest_packet(&None).unwrap();
        assert_eq!(packet_id, PacketId(33));
        assert_eq!(
            packet,
            ReadVariableSetXY {
                timestamp: 1.25,
                robot_mode: 7
            }
        );

        let (packet_id, packet) = reader2.get_latest_packet(&None).unwrap();
        assert_eq!(packet_id, PacketId(33));
        assert_eq!(
            packet,
            ReadVariableSetYZ {
                robot_mode: 7,
                payload: 7.5
            }
        );
    }
}
