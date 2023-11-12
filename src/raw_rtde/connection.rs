use std::mem::{discriminant, Discriminant};
use std::net::SocketAddr;
use std::io::{Cursor, Read, Write, ErrorKind};
use std::ops::Add;
use std::sync::Arc;
use std::time::{Duration, Instant};

use byteorder::{BigEndian, ReadBytesExt, WriteBytesExt};
use mio::{event, Poll, Token, Interest, Events, Waker};
use mio::net::TcpStream;

use crate::raw_rtde::packet_data::{RTDEData, ControllerVersion, IoSetup};

const RTDE_SEND_TIMEOUT: Duration = Duration::from_millis(50);
const RTDE_WAIT_FOR_REPLY_TIMEOUT: Duration = Duration::from_millis(2000);

enum RTDECommand {
    ConfigureInputs = b'I' as isize,
    ConfigureOutputs = b'O' as isize,
    Start = b'S' as isize,
    Pause = b'P' as isize,
    RtdeDataPacket = b'U' as isize,
    RequestProtocolVersion = b'V' as isize,
    GetControllerVersion = b'v' as isize,
}
const WOKEN_BEFORE_READ_PSEUDO_CMD: u8 = 255;

/// A low-level class to control RTDE at the level of single packets.
/// 
/// Except for tests it's recommended to use `RawRTDE` to send RTDE over TCP/IP,
/// as the UR robot expects to receive it.
pub struct GenericRawRTDE<T: Read + Write + event::Source> {
    stream: T,
    poll: Poll,
    waker: Arc<Waker>,
    wait_for_reply_timeout: Duration,  // Overridden in tests.
}

/// A raw RTDE connection over TCP/IP.
pub type RawRTDE = GenericRawRTDE<TcpStream>;


const RTDE_VERSION: u16 = 2;
const HEADER_SIZE: usize = 3;

const SOCKET_WAIT_TOKEN: Token = Token(0);
const WAKE_TOKEN: Token = Token(1);

enum ReadResult {
    ReadData,
    WokenBeforeRead,
}

/// Common method of a real RawRTDE and FakeRawRTDE.
pub trait RawRTDETrait
{
    /// Informs the robot to start sending data. `configure_outputs` need to be called first.
    fn start_robot_to_host_transmition(&mut self) -> Result<(), String>;
    /// Asks the robot to stop sending data.
    fn pause_robot_to_host_transmition(&mut self) -> Result<(), String>;
    /// Specifies which RTDE variables we want to write to.
    /// 
    /// There can be several RTDE connections to the robot, but each variable can be
    /// written to by at most one connection. 
    fn configure_inputs<U: 'static + AsRef<str>>(&mut self, variables: &[U]) -> Result<IoSetup, String>;
    /// Specifies which RTDE variables we want to receive.
    /// 
    /// There can be several RTDE connections to the robot, and each variable can be
    /// read to by multiple of them. 
    fn configure_outputs<U: 'static + AsRef<str>>(&mut self, frequency: f64, variables: &[U]) -> Result<IoSetup, String>;
    /// Sends a packet to the robot. `configure_inputs` needs to be called first.
    fn send_robot_input(&mut self, io_setup: &IoSetup, input_data: &[RTDEData]) -> Result<(), String>;

    /// Waits for a packet send by the robot.
    /// 
    /// One needs to call `configure_outputs` and `start_robot_to_host_transmition` to receive such packets.
    /// 
    /// May return None is someone used the waker from `get_wait_for_robot_output_waker` to interrupt the call.
    fn wait_for_robot_output(&mut self, io_setup: &IoSetup) -> Result<Option<Vec<RTDEData>>, String>;

    /// Returns a closure that can be called to interrupt a wait_for_robot_output (happening in another thread).
    fn get_wait_for_robot_output_interrupter(&self) -> Box<dyn Fn() -> Result<(), String> + Send + 'static>;
}

impl GenericRawRTDE<TcpStream> {
    /// Connect to a given TCP/IP port and negotiate the protocol version.
    pub fn connect(addr: SocketAddr) -> Result<Self, String> {
        let stream = TcpStream::connect(addr).map_err(|e| format!("Error connecting to RTDE: {}", e))?;

        let mut result = GenericRawRTDE::new(stream)?;
        result.negotiate_protocol_version()?;
        Ok(result)
    }
}

impl<T: Read + Write + event::Source> GenericRawRTDE<T> {
    /// Create a new connection for a given stream.
    /// 
    /// This is a low-level call (for most purposes one should use `RawRTDE::connect`) and protocol version
    /// negotation is not performed.
    pub fn new(stream: T) -> Result<Self, String> {
        let poll = Poll::new().map_err(|e| format!("Error starting RTDE - when create a Poll: {e}"))?;

        let waker = Arc::new(Waker::new(poll.registry(), WAKE_TOKEN).map_err(|e| format!("Error creating waker: {e}"))?);

        Ok(GenericRawRTDE {
            stream,
            poll,
            waker,
            wait_for_reply_timeout: RTDE_WAIT_FOR_REPLY_TIMEOUT,
        })
    }

    /// Asks the control about its version.
    pub fn get_controller_version(&mut self) -> Result<ControllerVersion, String> {
        let request = Self::encode_controller_version_request()?;
        self.send_packet(&request)?;
        let reply = self.wait_for_reply(RTDECommand::GetControllerVersion as u8)?;
        Self::decode_controller_version_reply(reply)
    }

    /// Negotiates the protocol version.
    /// 
    /// Needs to be done before sending other packets. `RawRTDE::connect` does it automatically.
    pub fn negotiate_protocol_version(&mut self) -> Result<(), String> {
        let request = Self::encode_protocol_version_request(RTDE_VERSION)?;
        self.send_packet(&request)?;
        let reply = self.wait_for_reply(RTDECommand::RequestProtocolVersion as u8)?;
        let protocol_ok = Self::decode_protocol_version_reply(reply)?;

        match protocol_ok {
            true => Ok(()),
            false => Err(format!("Robot did not accept RTDE protocol version {}", RTDE_VERSION)),
        }
    }

    fn encode_protocol_version_request(version: u16) -> Result<Vec<u8>, String> {
        let mut payload: Vec<u8> = vec![];
        payload.write_u16::<BigEndian>(version).unwrap();
        Self::encode_packet(RTDECommand::RequestProtocolVersion as u8, payload)
    }

    fn decode_protocol_version_reply(payload: Vec<u8>) -> Result<bool, String>{
        let mut rdr = Cursor::new(payload);
        let protocol_ok = rdr.read_u8().map_err(|_e| "Protocol version reply too short")? != 0;
        Ok(protocol_ok)
    }

    fn encode_controller_version_request() -> Result<Vec<u8>, String> {
        Self::encode_packet(RTDECommand::GetControllerVersion as u8, vec![])
    }

    fn decode_controller_version_reply(payload: Vec<u8>) -> Result<ControllerVersion, String>{
        let mut rdr = Cursor::new(payload);
        let major = rdr.read_i32::<BigEndian>().map_err(|_e| "Controller version reply too short")?;
        let minor = rdr.read_i32::<BigEndian>().map_err(|_e| "Controller version reply too short")?;
        let bugfix = rdr.read_i32::<BigEndian>().map_err(|_e| "Controller version reply too short")?;
        let build = rdr.read_i32::<BigEndian>().map_err(|_e| "Controller version reply too short")?;
        Ok(ControllerVersion{
            major,
            minor,
            bugfix,
            build,
        })
    }

    fn encode_payloadless_request(command: RTDECommand) -> Result<Vec<u8>, String> {
        Self::encode_packet(command as u8, vec![])
    }

    fn decode_status_only_reply(payload: Vec<u8>) -> Result<bool, String> {
        let mut rdr = Cursor::new(payload);
        let accepted = rdr.read_u8().map_err(|_e| "Controller version reply too short")?;
        Ok(accepted != 0)
    }

    fn encode_configure_inputs_request<U: AsRef<str>>(variables: &[U]) -> Result<Vec<u8>, String> {
        let variables = variables.into_iter().map(|v| v.as_ref()).collect::<Vec<&str>>();
        if !variables.iter().all(|v| !v.contains(',')) {
            return Err(format!("Comma can't be part of variable name in {:?}", variables));
        }
        let payload_as_string = variables.join(",");
        let payload = payload_as_string.as_bytes().to_vec();
        Self::encode_packet(RTDECommand::ConfigureInputs as u8, payload)
    }

    fn encode_configure_outputs_request<U: AsRef<str>>(frequency: f64, variables: &[U]) -> Result<Vec<u8>, String> {
        let variables = variables.into_iter().map(|v| v.as_ref()).collect::<Vec<&str>>();
        if !variables.iter().all(|v| !v.contains(',')) {
            return Err(format!("Comma can't be part of variable name in {:?}", variables));
        }
        let mut payload: Vec<u8> = vec![];
        payload.write_f64::<BigEndian>(frequency).unwrap();
        payload.extend(variables.join(",").as_bytes());

        Self::encode_packet(RTDECommand::ConfigureOutputs as u8, payload)
    }

    fn map_type_name_to_discriminant(type_name: &str) -> Result<Discriminant<RTDEData>, String> {
        let default_value = RTDEData::default_for_type_name(type_name)?;
        Ok(discriminant(&default_value))
    }

    fn decode_configure_io_reply<U: AsRef<str>>(payload: Vec<u8>, variables: &[U]) -> Result<IoSetup, String>{
        let variables = variables.into_iter().map(|v| v.as_ref()).collect::<Vec<&str>>();
        let mut rdr = Cursor::new(payload);
        let recipe_id: u8 = rdr.read_u8().map_err(|_e| "Inputs setup reply too short")?.into();
        let payload = rdr.into_inner();
        let string = String::from_utf8(payload[1..].to_vec()).map_err(|_e| format!("Inputs reply contains non-UTF8 chars: {:?}", &payload[1..]))?;
        let type_names: Vec<String> = string.split(",").map(|s| String::from(s)).collect();

        if let Some(offender) = type_names.iter().position(|t| t == "NOT_FOUND") {
            return Err(format!("Variable {} not found", variables[offender]));
        }
        if let Some(offender) = type_names.iter().position(|t| t == "IN_USE") {
            return Err(format!("Variable {} is already in use by another connection", variables[offender]));
        }

        let types: Vec<Discriminant<RTDEData>> = type_names.into_iter().map(|n| Self::map_type_name_to_discriminant(&n)).collect::<Result<Vec<_>,_>>()?;

        Ok(IoSetup {
            recipe_id,
            types,
        })
    }

    fn encode_robot_input_packet(io_setup: &IoSetup, input_data: &[RTDEData]) -> Result<Vec<u8>, String> {
        assert_eq!(io_setup.types.len(), input_data.len());

        let mut payload: Vec<u8> = vec![];
        payload.write_u8(io_setup.recipe_id).unwrap();
        for (i, one_input) in input_data.iter().enumerate() {
            assert_eq!(discriminant(one_input), io_setup.types[i]);
            one_input.serialize(&mut payload);
        }
        Self::encode_packet(RTDECommand::RtdeDataPacket as u8, payload)
    }

    fn decode_robot_output_packet(payload: &[u8], io_setup: &IoSetup) -> Result<Vec<RTDEData>, String> {
        let mut rdr = Cursor::new(payload);
        let recipe_id = rdr.read_u8().map_err(|_e| "Data packet received too short")?;
        if recipe_id != io_setup.recipe_id {
            return Err(format!("Received recipie id {} instead of {}", recipe_id, io_setup.recipe_id));
        }

        let result = io_setup.types
            .iter()
            .map(|current_type| RTDEData::parse(&mut rdr, *current_type))
            .collect::<Result<Vec<RTDEData>, String>>()?;

        Ok(result)
    }

    fn encode_packet(command: u8, payload: Vec<u8>) -> Result<Vec<u8>, String> {
        let packet_size: u16 = (HEADER_SIZE + payload.len()).try_into().map_err(|_e| "Payload too large")?;

        let mut result = vec![];
        result.write_u16::<BigEndian>(packet_size).unwrap();
        result.write_u8(command).unwrap();
        result.extend(payload);
        Ok(result)
    }

    fn send_packet(&mut self, packet: &[u8]) -> Result<(), String> {
        log::debug!("Sending packet {:x?}", packet);
        let deadline = Instant::now().add(RTDE_SEND_TIMEOUT);
        self.mio_write_all(packet, deadline)
    }

    fn mio_write_all(&mut self, packet: &[u8], deadline: Instant) -> Result<(), String>{
        let mut events = Events::with_capacity(16);
        self.poll.registry().register(&mut self.stream, SOCKET_WAIT_TOKEN, Interest::WRITABLE).map_err(|e| format!("Error sending data - when registering poll event: {e}"))?;

        let mut result = Ok(());
        let mut written = 0;
        while written < packet.len() {
            match self.stream.write(&packet[written..]) {
                Ok(n) => written += n,
                Err(ref e) if e.kind() == ErrorKind::Interrupted => {}
                Err(ref e) if e.kind() == ErrorKind::WouldBlock => {
                    self.poll.poll(&mut events, Some(deadline.duration_since(Instant::now()))).map_err(|e| format!("Error sending data - when polling the socket: {e}"))?;
                }
                Err(e) => {
                    result =  Err(format!("Error writing to socket: {e}"));
                    break;
                }
            }
        }
        self.poll.registry().deregister(&mut self.stream).map_err(|e| format!("Error sending data - deregistering poll: {e}"))?;
        result
    }

    fn wait_for_reply_interruptible(&mut self, command: u8, allow_waker: bool) -> Result<Option<Vec<u8>>, String> {
        let deadline = Instant::now().add(self.wait_for_reply_timeout);
        loop {
            log::debug!("Waiting for reply for command {}", command);
            let (packet_cmd, payload) = self.wait_for_packet(deadline, allow_waker).map_err(|e| format!("Error waiting for reply to command {command}: {e}"))?;
            match packet_cmd {
                cmd if cmd == command => return Ok(Some(payload)),
                cmd if cmd == WOKEN_BEFORE_READ_PSEUDO_CMD => return Ok(None),
                cmd if cmd == RTDECommand::RtdeDataPacket as u8 => continue,  // Ignore data packets
                _ => return Err(format!("Unexpected packet type {}", packet_cmd))
            }
        }
    }

    fn wait_for_reply(&mut self, command: u8) -> Result<Vec<u8>, String> {
        self.wait_for_reply_interruptible(command, false).map(|opt_payload| opt_payload.expect("When allow_waker is None, the payload must be set"))
    }

    fn wait_for_packet(&mut self, deadline: Instant, allow_waker: bool) -> Result<(u8, Vec<u8>), String> {
        let mut header = [0u8; HEADER_SIZE];
        match self.mio_read_exact(&mut header, deadline, allow_waker)? {
            ReadResult::ReadData => { /* proceed */},
            ReadResult::WokenBeforeRead => { return Ok((WOKEN_BEFORE_READ_PSEUDO_CMD, Vec::<u8>::new())); },
        }
        log::debug!("Got packet header: {:x?}", &header);

        let (cmd, payload_size) = Self::decode_header(header)?;
        let mut payload: Vec<u8> = vec![0; payload_size.into()];
        self.mio_read_exact(&mut payload, deadline, false)?;
        log::debug!("Got packet payload: {:x?}", &payload);

        Ok((cmd, payload))
    }

    fn mio_read_exact(&mut self, buf: &mut [u8], deadline: Instant, allow_waker: bool) -> Result<ReadResult, String> {
        let mut events = Events::with_capacity(16);
        self.poll.registry().register(&mut self.stream, SOCKET_WAIT_TOKEN, Interest::READABLE).map_err(|e| format!("Error receiving data - when registering poll event: {e}"))?;

        let mut result = Ok(ReadResult::ReadData);
        let mut read = 0;
        while read < buf.len() {
            match self.stream.read(&mut buf[read..]) {
                Ok(n) => read += n,
                Err(ref e) if e.kind() == ErrorKind::Interrupted => {}
                Err(ref e) if e.kind() == ErrorKind::WouldBlock => {
                    let timeout = deadline.duration_since(Instant::now());
                    if timeout.is_zero() {
                        return Err(String::from("Timeout"));
                    }
                    self.poll.poll(&mut events, Some(timeout)).map_err(|e| format!("Error sending data - when polling the socket: {e}"))?;
                    if allow_waker && read == 0 && Self::_was_waker_used(&events) {
                        result = Ok(ReadResult::WokenBeforeRead);
                        break;
                    }
                }
                Err(e) => {
                    result =  Err(format!("Error reading from socket: {e}"));
                    break;
                }
            }
        }
        self.poll.registry().deregister(&mut self.stream).map_err(|e| format!("Error receiving data - deregistering poll: {e}"))?;
        result
    }

    fn _was_waker_used(events: &Events) -> bool {
        for event in events {
            if event.token() == WAKE_TOKEN {
                return true;
            }
        }

        false
    }

    fn decode_header(header: [u8; HEADER_SIZE]) -> Result<(u8, u16), String> {
        let mut rdr = Cursor::new(header);
        let packet_size = rdr.read_u16::<BigEndian>().unwrap();
        let command = rdr.read_u8().unwrap();

        let payload_size: u16 = packet_size.checked_sub(HEADER_SIZE as u16).ok_or_else(
            || format!("Illegal packet size {}", packet_size)
        )?;

        Ok((command, payload_size))
    }

}


impl<T: Read + Write + event::Source> RawRTDETrait for GenericRawRTDE<T> {
    fn configure_inputs<U: AsRef<str>>(&mut self, variables: &[U]) -> Result<IoSetup, String>{
        let request = Self::encode_configure_inputs_request(variables)?;
        self.send_packet(&request)?;
        let reply = self.wait_for_reply(RTDECommand::ConfigureInputs as u8)?;
        Self::decode_configure_io_reply(reply, variables)
    }

    fn configure_outputs<U: AsRef<str>>(&mut self, frequency: f64, variables: &[U]) -> Result<IoSetup, String>{
        let request = Self::encode_configure_outputs_request(frequency, variables)?;
        self.send_packet(&request)?;
        let reply = self.wait_for_reply(RTDECommand::ConfigureOutputs as u8)?;
        Self::decode_configure_io_reply(reply, variables)
    }

    fn send_robot_input(&mut self, io_setup: &IoSetup, input_data: &[RTDEData]) -> Result<(), String> {
        let request = Self::encode_robot_input_packet(io_setup, input_data)?;
        self.send_packet(&request)
    }

    fn wait_for_robot_output(&mut self, io_setup: &IoSetup) -> Result<Option<Vec<RTDEData>>, String> {
        let opt_reply = self.wait_for_reply_interruptible(RTDECommand::RtdeDataPacket as u8, true)?;
        match opt_reply {
            Some(reply) => Ok(Some(Self::decode_robot_output_packet(&reply, io_setup)?)),
            None => Ok(None),
        }
    }

    fn get_wait_for_robot_output_interrupter(&self) -> Box<dyn Fn() -> Result<(), String> + Send + 'static> {
        let waker = self.waker.clone();
        Box::new(move || waker.wake().map_err(|e| format!("Error waking the RTDE thread: {e}")))
    }

    fn start_robot_to_host_transmition(&mut self) -> Result<(), String>{
        let request = Self::encode_payloadless_request(RTDECommand::Start)?;
        self.send_packet(&request)?;
        let reply = self.wait_for_reply(RTDECommand::Start as u8)?;
        match Self::decode_status_only_reply(reply)? {
            true => Ok(()),
            false => Err("Robot did not accept to start the transmition".to_string()),
        }
    }

    fn pause_robot_to_host_transmition(&mut self) -> Result<(), String> {
        let request = Self::encode_payloadless_request(RTDECommand::Pause)?;
        self.send_packet(&request)?;
        let reply = self.wait_for_reply(RTDECommand::Pause as u8)?;
        match Self::decode_status_only_reply(reply)? {
            true => Ok(()),
            false => Err("Robot did not accept to pause the transmition".to_string()),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::thread::{self, sleep};

    use mio::net::UnixStream;

    use super::*;

    #[ctor::ctor]
    fn init() {
        let _ = env_logger::builder().is_test(true).try_init();
    }

    #[test]
    fn rtde_data_serializes() {
        let data = RTDEData::Uint64(255 + 4*256);
        let mut buffer = vec![];

        data.serialize(&mut buffer);

        assert_eq!(buffer, [0, 0, 0, 0, 0, 0, 4, 255]);
    }

    #[test]
    fn encodes_simple_packet() {
        let result = GenericRawRTDE::<TcpStream>::encode_packet(32, vec![1, 2, 3, 4]);
        assert_eq!(result, Ok(vec![
            0, 7, // u16 BigEndian size
            32, // u8 command
            1, 2, 3, 4,  // Payload
        ]));
    }

    #[test]
    fn encodes_protocol_version() {
        let result = GenericRawRTDE::<TcpStream>::encode_protocol_version_request(2);
        assert_eq!(result, Ok(vec![
            0, 5, // u16 BigEndian size
            86, // u8 command
            0, 2,  // Version in BE u16.
        ]));
    }

    #[test]
    fn negotiates_protocol_version() {
        let (mut tester_end, sut_end) = UnixStream::pair().unwrap();
        tester_end.write_all(&[
            0, 4,  // u16 BigEndian packet size
            b'V' as u8,  // Command
            1,  // Command accepted
        ]).unwrap();

        let mut rtde = GenericRawRTDE::new(sut_end).unwrap();

        assert_eq!(rtde.negotiate_protocol_version(), Ok(()));
    }

    #[test]
    fn queries_controller_version() {
        let (mut tester_end, sut_end) = UnixStream::pair().unwrap();
        tester_end.write_all(&[
            0, 19,  // u16 BigEndian packet size
            b'v' as u8,  // Command
            0, 0, 0, 1,  // Major
            0, 0, 0, 2,  // Minor
            0, 0, 0, 3,  // Bugfix
            0, 0, 0, 4,  // Build
        ]).unwrap();

        let mut rtde = GenericRawRTDE::new(sut_end).unwrap();

        let version = rtde.get_controller_version().expect("Function failed");
        assert_eq!(version.to_string(), "1.2.3.4");
    }

    #[test]
    fn handles_data_received_late() {
        let (mut tester_end, sut_end) = UnixStream::pair().unwrap();

        let mut rtde = GenericRawRTDE::new(sut_end).unwrap();

        thread::spawn(move || {
            sleep(Duration::from_millis(10));
            tester_end.write_all(&[
                0, 19,  // u16 BigEndian packet size
                b'v' as u8,  // Command
                0, 0, 0, 1,  // Major
                0, 0, 0, 2,  // Minor
                0, 0, 0, 3,  // Bugfix
                0, 0, 0, 4,  // Build
            ]).unwrap();
        });

        let version = rtde.get_controller_version().expect("Function failed");
        assert_eq!(version.to_string(), "1.2.3.4");
    }

    #[test]
    fn handles_no_data_received() {
        let (_tester_end, sut_end) = UnixStream::pair().unwrap();

        let mut rtde = GenericRawRTDE::new(sut_end).unwrap();
        rtde.wait_for_reply_timeout = Duration::from_millis(5);

        match rtde.get_controller_version() {
            Err(e) => assert!(e.contains("Timeout"), "{e}"),
            Ok(_) => assert!(false, "Request should have failed"),
        }
    }

    #[test]
    fn test_configure_inputs() {
        let (mut tester_end, sut_end) = UnixStream::pair().unwrap();
        let types_string = "INT32,UINT8";
        tester_end.write_all(&[
            0, (4 + types_string.len()).try_into().unwrap(),  // u16 BigEndian packet size
            b'I' as u8,  // Command
            23,  // Recipie ID
        ]).unwrap();
        tester_end.write_all(types_string.as_bytes()).unwrap();

        let mut rtde = GenericRawRTDE::new(sut_end).unwrap();

        let inputs_setup = rtde.configure_inputs(&["a", "b"]).expect("Function failed");
        assert_eq!(inputs_setup.recipe_id, 23);
        assert_eq!(inputs_setup.types, &[discriminant(&RTDEData::Int32(0)), discriminant(&RTDEData::Uint8(0))]);
    }

    #[test]
    fn test_start_robot_to_host_transmition() {
        let (mut tester_end, sut_end) = UnixStream::pair().unwrap();
        tester_end.write_all(&[
            0, 4,  // u16 BigEndian packet size
            b'S' as u8,  // Command
            1, // Accepted
        ]).unwrap();

        let mut rtde = GenericRawRTDE::new(sut_end).unwrap();

        rtde.start_robot_to_host_transmition().expect("Function failed");
        let mut written = Vec::<u8>::new();
        assert_eq!(tester_end.read_to_end(&mut written).map_err(|e| e.kind()), Err(std::io::ErrorKind::WouldBlock));
        assert_eq!(written, &[
            0, 3,  // u16 BigEndiant packet size
            b'S', // Command
        ]); 
    }

    #[test]
    fn test_pause_robot_to_host_transmition() {
        let (mut tester_end, sut_end) = UnixStream::pair().unwrap();
        tester_end.write_all(&[
            0, 4,  // u16 BigEndian packet size
            b'P' as u8,  // Command
            1, // Success
        ]).unwrap();

        let mut rtde = GenericRawRTDE::new(sut_end).unwrap();

        rtde.pause_robot_to_host_transmition().expect("Function failed");
        let mut written = Vec::<u8>::new();
        assert_eq!(tester_end.read_to_end(&mut written).map_err(|e| e.kind()), Err(std::io::ErrorKind::WouldBlock));
        assert_eq!(written, &[
            0, 3,  // u16 BigEndiant packet size
            b'P', // Command
        ]); 
    }

    #[test]
    fn test_send_robot_inputs() {
        let (mut tester_end, sut_end) = UnixStream::pair().unwrap();
        let input_data = [
            RTDEData::Uint32(1),
            RTDEData::Vector3Int32([2, 3, 4]),
        ];
        let io_setup = IoSetup{
            recipe_id: 23,
            types: vec![discriminant(&input_data[0]), discriminant(&input_data[1])],
        };

        let mut rtde = GenericRawRTDE::new(sut_end).unwrap();

        rtde.send_robot_input(&io_setup, &input_data).expect("Function failed");
        let mut written = Vec::<u8>::new();
        assert_eq!(tester_end.read_to_end(&mut written).map_err(|e| e.kind()), Err(std::io::ErrorKind::WouldBlock));
        assert_eq!(written, &[
            0, 20,  // u16 BigEndiant packet size
            b'U', // Command
            23,  // Recipie ID
            0, 0, 0, 1,  //  First field (UIN32)
            0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 4,  //  Second field (VECTOR3INT32)
        ]); 
    }

    #[test]
    fn test_configure_outputs() {
        let (mut tester_end, sut_end) = UnixStream::pair().unwrap();
        let types_string = "INT32,UINT8";
        tester_end.write_all(&[
            0, (4 + types_string.len()).try_into().unwrap(),  // u16 BigEndian packet size
            b'O' as u8,  // Command
            45,  // Recipie ID
        ]).unwrap();
        tester_end.write_all(types_string.as_bytes()).unwrap();

        let mut rtde = GenericRawRTDE::new(sut_end).unwrap();
        
        let inputs_setup = rtde.configure_outputs(125f64, &["a", "b"]).expect("Function failed");
        assert_eq!(inputs_setup.recipe_id, 45);
        assert_eq!(inputs_setup.types, &[discriminant(&RTDEData::Int32(0)), discriminant(&RTDEData::Uint8(0))]);

        let mut written = Vec::<u8>::new();
        assert_eq!(tester_end.read_to_end(&mut written).map_err(|e| e.kind()), Err(std::io::ErrorKind::WouldBlock));
        assert_eq!(written, &[
            0, 14,  // u16 BigEndian packet size
            b'O', // Command
            64, 95, 64, 0, 0, 0, 0, 0, //  Frequency 
            b'a', b',', b'b',  // Variable names 
        ]); 
    }

    #[test]
    fn test_parse_outpute_packet() {
        let (mut tester_end, sut_end) = UnixStream::pair().unwrap();
        tester_end.write_all(&[
            0, 9,  // u16 BigEndian packet size
            b'U' as u8,  // Command
            45,  // Recipie ID
            7,  // UINT8 value
            0, 0, 0, 23,  // UINT32 value
        ]).unwrap();

        let io_setup = IoSetup{
            recipe_id: 45,
            types: vec![
                discriminant(&RTDEData::Uint8(0)),
                discriminant(&RTDEData::Int32(0)),
            ],
        };

        let mut rtde = GenericRawRTDE::new(sut_end).unwrap();

        let result = rtde.wait_for_robot_output(&io_setup).expect("Function failed");

        assert_eq!(result, Some(vec![
            RTDEData::Uint8(7),
            RTDEData::Int32(23),
        ]))
    }

    #[test]
    fn can_interrupt_wait() {
        let (_tester_end, sut_end) = UnixStream::pair().unwrap();

        let io_setup = IoSetup{
            recipe_id: 45,
            types: vec![
                discriminant(&RTDEData::Uint8(0)),
                discriminant(&RTDEData::Int32(0)),
            ],
        };
        let mut rtde = GenericRawRTDE::new(sut_end).unwrap();
        let interrupter = rtde.get_wait_for_robot_output_interrupter();

        thread::spawn(move || {
            sleep(Duration::from_millis(10));
            interrupter().unwrap();
        });

        let result = rtde.wait_for_robot_output(&io_setup).expect("Function failed");
        assert_eq!(result, None);
    }

}
