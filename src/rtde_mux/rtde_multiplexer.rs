use std::collections::{BTreeSet, BTreeMap, HashMap};
use std::collections::btree_map::Entry::{Occupied, Vacant};
use std::iter::zip;
use std::marker::PhantomData;
use std::sync::{Arc, Mutex};

use crate::raw_rtde::{RTDEData, RawRTDETrait};
use crate::rtde_thread::{RTDEThread, RTDEThreadTrait};
use crate::rtde_mux::{RTDEReadVariableSet, RTDEWriteVariableSet, RTDEReader, RTDEWriter, RTDEWriterSharedData};


/// Builds a multiplexed connection with potentially multiple RTDEReaders and RTDEWriters.
/// 
/// Since a UR robot has a limited number of connections available (3 or 6 depending on firmware
/// version), one usually wants to reuse the same connection for logicaly spearate blocks. The
/// multiplexer allows this, additionally providing an easy to use and strongly typed API based on
/// `RTDE(Read|Write)VariableSet``.
pub struct RTDEMultiplexedConnectionBuilder<T>
    where T: 'static + RawRTDETrait + Send
{
    raw_rtde: T,
    read_variables: BTreeSet<String>,
    write_variables: BTreeMap<String, RTDEData>,
    read_frequency: f64,
    cookie: usize,
}

/// A promise that once the connection is established, a writer will be available.
pub struct RTDEWriterPromise<U: RTDEWriteVariableSet> {
    cookie: usize,  // Best effort check that the promise is from the right RTDEMultiplexdConnectionBuilder.
    phantom: PhantomData<U>,  // Zero-sized marker that makes sure the generated writer will be of a correct type.
}

/// A promise that once the connection is established, a reader will be available.
pub struct RTDEReaderPromise<U: RTDEReadVariableSet> {
    cookie: usize,  // Best effort check that the promise is from the right RTDEMultiplexdConnectionBuilder.
    phantom: PhantomData<U>,  // Zero-sized marker that makes sure the generated reader will be of a correct type.
}
impl<T> RTDEMultiplexedConnectionBuilder<T>
    where T: 'static + RawRTDETrait + Send
{
    /// Create a new builder.
    pub fn new(
        raw_rtde: T,
        read_frequency: f64,
    ) -> RTDEMultiplexedConnectionBuilder<T> {
        let mut result = RTDEMultiplexedConnectionBuilder{
            raw_rtde,
            read_variables: BTreeSet::new(),
            write_variables: BTreeMap::new(),
            read_frequency: read_frequency,
            cookie: 0,
        };
        result.cookie = &result.read_frequency as *const f64 as usize;  // Best effort unique ID of this multiplexer.
        result
    }

    /// Requests a reader (i.e. robot to host transmition) for a given variable set.
    /// 
    /// All readers must be requested before the connection is made, but will be available
    /// only after the connection. Thus, this method returns only a "promise" that can be
    /// transformed into a reader after the connection.
    pub fn add_read_variable_set<U: RTDEReadVariableSet>(&mut self) -> Result<RTDEReaderPromise<U>, String> {
        for var in U::get_variables() {
            self.read_variables.insert(var);
        }

        Ok(RTDEReaderPromise{
            cookie: self.cookie,
            phantom: PhantomData{},
        })
    }

    /// Requests a writer (i.e. host to robot transmition) for a given variable set.
    /// 
    /// All writer must be requested before the connection is made, but will be available
    /// only after the connection. Thus, this method returns only a "promise" that can be
    /// transformed into a writer after the connection.
    pub fn add_write_variable_set<U: RTDEWriteVariableSet>(&mut self, initial_values: &U) -> Result<RTDEWriterPromise<U>, String> {
        let rtde_data = initial_values.convert_into_rtde_data();

        for (var, value) in zip(U::get_variables().iter(), rtde_data.into_iter()) {
            let entry = self.write_variables.entry(var.clone());
            if let Occupied(oe) = entry {
                let current_entry = oe.get();
                if *current_entry != value {
                    return Err(String::from(format!("Variable {var} initialized again to a differnent value ({value:?} vs {current_entry:?})")));
                }
            } else if let Vacant(ve) = entry {
                ve.insert(value);
            }
        }

        Ok(RTDEWriterPromise{
            cookie: self.cookie,
            phantom: PhantomData{},
        })
    }

    /// Connect to the robot.
    /// 
    /// This requests all the variables for all the readers and writers requested - no need to
    /// specify variables manually.
    /// 
    /// Establishing the connection will send a packet of data with the initial value of variables.
    pub fn connect(self) -> Result<RTDEMultiplexedConnection, String> {
        let thread: Arc<dyn RTDEThreadTrait> = Arc::new(
            RTDEThread::start(
                self.raw_rtde,
                &self.write_variables.keys().map(String::from).collect::<Vec<String>>(),
                &self.read_variables.iter().map(String::from).collect::<Vec<String>>(),
                self.read_frequency,
            )?
        );

        let writer_shared_data = RTDEWriterSharedData::new(
            &thread, &self.write_variables,
        );

        // BTreeSet doesn't seem to provide an easy way to get the index of an element, so let's build an index. 
        let var_names_to_index = self.read_variables.iter().enumerate().map(|(i, var_name)| (var_name.clone(), i)).collect();

        writer_shared_data.lock().unwrap().send_initial_values()?;

        Ok(RTDEMultiplexedConnection {
            cookie: self.cookie, 
            rtde_thread: thread,
            var_names_to_index,
            writer_shared_data,
        })
    }
}

/// A multiplexed connection. Allows to get actual readers and writer and to disconnect it.
pub struct RTDEMultiplexedConnection {
    cookie: usize,
    rtde_thread: Arc<dyn RTDEThreadTrait>,  // Placed in SharedData so that the writes are ordered.
    var_names_to_index: HashMap<String, usize>,
    writer_shared_data: Arc<Mutex<RTDEWriterSharedData>>,
}

impl RTDEMultiplexedConnection {
    /// Change a promise of a reader into an actual reader.
    /// 
    /// See `RTDEMultiplexedConnectionBuilder.add_read_variable_set` for details.
    pub fn get_reader<U: RTDEReadVariableSet>(&self, promise: RTDEReaderPromise<U>) -> Result<RTDEReader<U>, String> {
        if promise.cookie != self.cookie {
            return Err(String::from(format!("The promise object is from a different RTDEMultiplexedConnection")));
        }

        RTDEReader::new(
            &self.rtde_thread,
            &self.var_names_to_index,
        )
    }

    /// Change a promise of a writer into an actual writer.
    /// 
    /// See `RTDEMultiplexedConnectionBuilder.add_write_variable_set` for details.
    pub fn get_writer<U: RTDEWriteVariableSet>(&self, promise: RTDEWriterPromise<U>) -> Result<RTDEWriter<U>, String> {
        if promise.cookie != self.cookie {
            return Err(String::from(format!("The promise object is from a different RTDEMultiplexedConnection")));
        }

        Ok(RTDEWriter::new(&self.writer_shared_data))
    }

    /// Disconnect RTDE.
    /// 
    /// All readers and writers need to be disposed of, or this call will fail.
    pub fn disconnect(self) -> Result<(), String> {
        let writers_left = Arc::strong_count(&self.writer_shared_data) - 1;  // 1 reference from RTDEMultiplexedConnection.
        if writers_left > 0 {
            return Err(String::from(format!("Some writers are still active (count {writers_left}")))
        }

        let readers_left = Arc::strong_count(&self.rtde_thread) - 2;  // 2 references from RTDEMultiplexedConnection.
        if readers_left > 0 {
            return Err(String::from(format!("Some readers are still active (count {readers_left}")))
        }

        let mut rtde_thread = self.rtde_thread.clone();
        std::mem::drop(self);

        if let Some(my_thread) = Arc::get_mut(&mut rtde_thread) {
            my_thread.stop()?;
        } else {
            return Err(String::from(format!("Some references to RTDE thread are left - cannot stop it.")));
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests
{
    use super::*;
    use crate::raw_rtde::FakeRawRTDE;
    use crate::raw_rtde::RTDEData;
    use crate::rtde_mux::test_variable_sets::tests::{ReadVariableSetXY, ReadVariableSetYZ, WriteVariableSetAB, WriteVariableSetCB};

    #[ctor::ctor]
    fn init()
    {
        let _ = env_logger::builder().is_test(true).try_init();
    }

    #[test]
    fn sets_inputs_and_outputs() {
        let (fake, control) = FakeRawRTDE::new_fake_and_control();

        let mut mux_builder = RTDEMultiplexedConnectionBuilder::new(fake, 125.0);

        let ab_vars = WriteVariableSetAB{
            standard_digital_output_mask: 255,
            standard_analog_output_0: 1.25,
        };
        let cb_vars = WriteVariableSetCB{
            standard_digital_output: 0,
            standard_digital_output_mask: 255,
        };

        mux_builder.add_write_variable_set(&cb_vars).unwrap();
        mux_builder.add_write_variable_set(&ab_vars).unwrap();
        mux_builder.add_read_variable_set::<ReadVariableSetXY>().unwrap();
        mux_builder.add_read_variable_set::<ReadVariableSetYZ>().unwrap();

        let connection = mux_builder.connect().unwrap();

        connection.disconnect().unwrap();

        assert_eq!(control.lock().unwrap().input_io_setup.as_ref().unwrap().vars, vec!["standard_analog_output_0", "standard_digital_output", "standard_digital_output_mask"]);
        assert_eq!(control.lock().unwrap().output_io_setups[0].vars, vec!["payload", "robot_mode", "timestamp"]);

    }

    #[test]
    fn detects_initial_values_collision() {
        let (fake, _) = FakeRawRTDE::new_fake_and_control();

        let mut mux_builder = RTDEMultiplexedConnectionBuilder::new(fake, 125.0);

        let ab_vars = WriteVariableSetAB{
            standard_digital_output_mask: 127,
            standard_analog_output_0: 1.25,
        };
        let cb_vars = WriteVariableSetCB{
            standard_digital_output: 0,
            standard_digital_output_mask: 255,
        };

        mux_builder.add_write_variable_set(&cb_vars).unwrap();
        let conflict_add_result = mux_builder.add_write_variable_set(&ab_vars);
        assert!(matches!(conflict_add_result, Err(_)));
    }

    #[test]
    fn can_read_packets() {
        let (fake, control) = FakeRawRTDE::new_fake_and_control();

        let mut mux_builder = RTDEMultiplexedConnectionBuilder::new(fake, 125.0);

        let reader_xy_promise = mux_builder.add_read_variable_set::<ReadVariableSetXY>().unwrap();
        let reader_yz_promise = mux_builder.add_read_variable_set::<ReadVariableSetYZ>().unwrap();

        let connection = mux_builder.connect().unwrap();

        let reader_xy = connection.get_reader(reader_xy_promise).unwrap();
        let reader_yz = connection.get_reader(reader_yz_promise).unwrap();

        control.lock().unwrap().queue_repeating_output(0, &[
            RTDEData::Double(1.25),  // Payload - see last assert.
            RTDEData::Int32(7), // Robot mode - see last assert.
            RTDEData::Double(128.0),  // Timestamp - see last assert
        ]);

        let (_packet_id_xy, xy) = reader_xy.get_latest_packet(&None).unwrap();
        let (_packet_id_yz, yz) = reader_yz.get_latest_packet(&None).unwrap();

        std::mem::drop(reader_xy);
        std::mem::drop(reader_yz);
        connection.disconnect().unwrap();

        assert_eq!(xy, ReadVariableSetXY{ timestamp: 128.0, robot_mode: 7 });
        assert_eq!(yz, ReadVariableSetYZ{ robot_mode: 7, payload: 1.25 });

        assert_eq!(control.lock().unwrap().output_io_setups[0].vars, vec!["payload", "robot_mode", "timestamp"]);

    }

    #[test]
    fn can_write_packets() {
        let (fake, control) = FakeRawRTDE::new_fake_and_control();

        let mut mux_builder = RTDEMultiplexedConnectionBuilder::new(fake, 125.0);

        let ab_vars = WriteVariableSetAB{
            standard_digital_output_mask: 1,
            standard_analog_output_0: 1.0,
        };
        let cb_vars = WriteVariableSetCB{
            standard_digital_output: 1,
            standard_digital_output_mask: 1,
        };

        let writer_ab_promise = mux_builder.add_write_variable_set(&ab_vars).unwrap();
        let writer_cb_promise = mux_builder.add_write_variable_set(&cb_vars).unwrap();

        let connection = mux_builder.connect().unwrap();

        let writer_ab = connection.get_writer(writer_ab_promise).unwrap();
        let writer_cb = connection.get_writer(writer_cb_promise).unwrap();

        writer_ab.send(&WriteVariableSetAB{
            standard_digital_output_mask: 7,
            standard_analog_output_0: 7.0,
        }).unwrap();
        writer_cb.send(&WriteVariableSetCB{
            standard_digital_output: 15,
            standard_digital_output_mask: 15,
        }).unwrap();

        std::mem::drop(writer_ab);
        std::mem::drop(writer_cb);
        connection.disconnect().unwrap();

        {
            let control_data = &control.lock().unwrap();
            assert_eq!(control_data.input_io_setup.as_ref().unwrap().vars, vec![
                "standard_analog_output_0", "standard_digital_output", "standard_digital_output_mask"
            ]);
            assert_eq!(control_data.sent_input_packets, vec![
                vec![RTDEData::Double(1.0), RTDEData::Uint8(1), RTDEData::Uint8(1)],  // Initial value sent without any call to send().
                vec![RTDEData::Double(7.0), RTDEData::Uint8(1), RTDEData::Uint8(7)],
                vec![RTDEData::Double(7.0), RTDEData::Uint8(15), RTDEData::Uint8(15)],
            ])
        }
    }

}
