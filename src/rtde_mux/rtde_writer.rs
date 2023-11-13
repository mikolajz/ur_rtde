use crate::raw_rtde::RTDEData;
use crate::rtde_mux::{MuxWriteBuffer, RTDEWriteVariableSet};
use crate::rtde_thread::RTDEThreadTrait;
use std::collections::BTreeMap;
use std::marker::PhantomData;
use std::sync::{Arc, Mutex};

pub(crate) struct RTDEWriterSharedData {
    rtde_thread: Arc<dyn RTDEThreadTrait>, // Placed in SharedData so that the writes are ordered.
    write_buffer: MuxWriteBuffer,
}

impl RTDEWriterSharedData {
    pub(crate) fn new(
        rtde_thread: &Arc<dyn RTDEThreadTrait>,
        initial_values: &BTreeMap<String, RTDEData>,
    ) -> Arc<Mutex<RTDEWriterSharedData>> {
        Arc::new(Mutex::new(RTDEWriterSharedData {
            rtde_thread: rtde_thread.clone(),
            write_buffer: MuxWriteBuffer::new(initial_values),
        }))
    }

    pub(crate) fn send_initial_values(&self) -> Result<(), String> {
        let values = self.write_buffer.values();
        if !values.is_empty() {
            self.rtde_thread.send(&values)
        } else {
            Ok(())
        }
    }
}

/// A writer (i.e. host to robot transmition) of a specific set of RTDE variables.
pub struct RTDEWriter<U: RTDEWriteVariableSet>
where
    U: RTDEWriteVariableSet,
{
    write_mutex: Arc<Mutex<RTDEWriterSharedData>>, // Shared by all writers.
    variables: Vec<String>,
    phantom: PhantomData<U>,
}

impl<U> RTDEWriter<U>
where
    U: RTDEWriteVariableSet,
{
    pub(crate) fn new(shared_data: &Arc<Mutex<RTDEWriterSharedData>>) -> Self {
        RTDEWriter {
            write_mutex: shared_data.clone(),
            variables: U::get_variables(),
            phantom: PhantomData {},
        }
    }

    /// Sends a packet of data to the robot.
    pub fn send(&self, data: &U) -> Result<(), String> {
        let mut shared_data = self.write_mutex.lock().unwrap();
        shared_data
            .write_buffer
            .update(&self.variables, &data.convert_into_rtde_data())?;
        shared_data
            .rtde_thread
            .send(&shared_data.write_buffer.values())?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::rtde_mux::test_variable_sets::tests::{WriteVariableSetAB, WriteVariableSetCB};
    use crate::rtde_thread::FakeRTDEThread;

    use super::*;

    #[test]
    fn test_writes_from_two_writers() {
        let mut initial_values = BTreeMap::<String, RTDEData>::new();
        initial_values.insert(
            String::from("standard_analog_output_0"),
            RTDEData::Double(7.25),
        );
        initial_values.insert(String::from("standard_digital_output"), RTDEData::Uint8(0));
        initial_values.insert(
            String::from("standard_digital_output_mask"),
            RTDEData::Uint8(0),
        );

        let (fake_thread, fake_thread_control) = FakeRTDEThread::new();
        let fake_thread_ptr: Arc<dyn RTDEThreadTrait> = Arc::new(fake_thread);

        let shared_data = RTDEWriterSharedData::new(&fake_thread_ptr, &initial_values);

        let writer1 = RTDEWriter::<WriteVariableSetCB>::new(&shared_data);
        let writer2 = RTDEWriter::<WriteVariableSetAB>::new(&shared_data);

        writer1
            .send(&WriteVariableSetCB {
                standard_digital_output: 0x5,
                standard_digital_output_mask: 0xf,
            })
            .unwrap();

        assert_eq!(fake_thread_control.borrow().sent_packets.len(), 1);
        assert_eq!(
            fake_thread_control.borrow().sent_packets[0].payload,
            vec![
                RTDEData::Double(7.25),
                RTDEData::Uint8(0x5),
                RTDEData::Uint8(0xf),
            ]
        );

        writer2
            .send(&WriteVariableSetAB {
                standard_analog_output_0: 1.75,
                standard_digital_output_mask: 0x3f,
            })
            .unwrap();

        assert_eq!(fake_thread_control.borrow().sent_packets.len(), 2);
        assert_eq!(
            fake_thread_control.borrow().sent_packets[1].payload,
            vec![
                RTDEData::Double(1.75),
                RTDEData::Uint8(0x5),
                RTDEData::Uint8(0x3f),
            ]
        );
    }
}
