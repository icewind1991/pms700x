//#![no_std]

use core::marker::PhantomData;
use embedded_hal::serial::{Read, Write};

mod private {
    pub trait Sealed {}

    impl Sealed for super::Active {}

    impl Sealed for super::Passive {}

    impl Sealed for super::UnInitialized {}
}

pub trait Mode: private::Sealed {}

pub struct UnInitialized;

pub struct Active;

pub struct Passive;

impl Mode for Active {}

impl Mode for Passive {}

impl Mode for UnInitialized {}

const START_HEADER_1: u8 = 0x42;
const START_HEADER_2: u8 = 0x4d;

const MAX_RESPONSE_SIZE: usize = 32;

#[derive(Debug, Default)]
pub struct SensorData {
    /// PM1.0 concentration in µg/m³, corrected for standard atmosphere
    pub pm10: u16,
    /// PM25 concentration in µg/m³, corrected for standard atmosphere
    pub pm25: u16,
    /// PM10 concentration in µg/m³, corrected for standard atmosphere
    pub pm100: u16,
    /// PM1.0 concentration in µg/m³, in current atmosphere
    pub pm10_atmos: u16,
    /// PM25 concentration in µg/m³, in current atmosphere
    pub pm25_atmos: u16,
    /// PM10 concentration in µg/m³, in current atmosphere
    pub pm100_atmos: u16,
    /// Number of >0.3µm particles per 0.1L
    pub pm03_count: u16,
    /// Number of >0.5µm particles per 0.1L
    pub pm05_count: u16,
    /// Number of >1.0µm particles per 0.1L
    pub pm10_count: u16,
    /// Number of >2.5µm particles per 0.1L
    pub pm25_count: u16,
    /// Number of >5.0µm particles per 0.1L
    pub pm50_count: u16,
    /// Number of >10.0µm particles per 0.1L
    pub pm100_count: u16,
}

impl SensorData {
    pub fn from_raw(raw: &[u8; MAX_RESPONSE_SIZE]) -> Self {
        SensorData {
            pm10: u16::from_be_bytes([raw[4], raw[5]]),
            pm25: u16::from_be_bytes([raw[6], raw[7]]),
            pm100: u16::from_be_bytes([raw[8], raw[9]]),
            pm10_atmos: u16::from_be_bytes([raw[10], raw[11]]),
            pm25_atmos: u16::from_be_bytes([raw[12], raw[13]]),
            pm100_atmos: u16::from_be_bytes([raw[14], raw[15]]),
            pm03_count: u16::from_be_bytes([raw[16], raw[17]]),
            pm05_count: u16::from_be_bytes([raw[18], raw[19]]),
            pm10_count: u16::from_be_bytes([raw[20], raw[21]]),
            pm25_count: u16::from_be_bytes([raw[22], raw[23]]),
            pm50_count: u16::from_be_bytes([raw[24], raw[25]]),
            pm100_count: u16::from_be_bytes([raw[26], raw[27]]),
        }
    }
}

#[derive(Default)]
struct SensorReader {
    byte_offset: u8,
    length: u8,
    data: [u8; MAX_RESPONSE_SIZE],
}

pub struct Pms700X<Error, Serial: Read<u8, Error = Error> + Write<u8, Error = Error>, Mode> {
    serial: Serial,
    mode: PhantomData<Mode>,
    command_writer: CommandWriter,
    reader: SensorReader,
}

impl<Error, Serial: Read<u8, Error = Error> + Write<u8, Error = Error>>
    Pms700X<Error, Serial, UnInitialized>
{
    pub fn new(serial: Serial) -> Self {
        Pms700X {
            serial,
            mode: PhantomData,
            command_writer: CommandWriter::default(),
            reader: SensorReader::default(),
        }
    }
}

impl<Error, Serial: Read<u8, Error = Error> + Write<u8, Error = Error>, SensorMode: Mode>
    Pms700X<Error, Serial, SensorMode>
{
    fn send_command(
        &mut self,
        command: Command,
        data: u16,
        expect_answer: bool,
    ) -> nb::Result<(), Error> {
        if self.command_writer.command == Command::None {
            self.command_writer = CommandWriter::new(command, data);
        }

        self.command_writer.write(&mut self.serial)?;
        self.serial.flush()?;
        if expect_answer {
            self.reader.fill_data(&mut self.serial)?;
        }
        self.command_writer.command = Command::None;
        Ok(())
    }

    fn into_mode<NewMode: Mode>(self) -> Pms700X<Error, Serial, NewMode> {
        Pms700X {
            serial: self.serial,
            mode: PhantomData,
            command_writer: self.command_writer,
            reader: self.reader,
        }
    }

    /// Set the sensor into active mode
    pub fn into_active(mut self) -> Result<Pms700X<Error, Serial, Active>, Error> {
        nb::block!(self.send_command(Command::SetMode, 1, true))?;
        Ok(self.into_mode())
    }

    /// Set the sensor into passive mode
    ///
    /// Note that after setting the sensor into passive mode, you should wait about 30-50ms
    /// before trying to read the sensor data or the sensor will not respond
    pub fn into_passive(mut self) -> Result<Pms700X<Error, Serial, Passive>, Error> {
        nb::block!(self.send_command(Command::SetMode, 0, true))?;
        Ok(self.into_mode())
    }

    /// Set the sensor to sleep or wake it up
    ///
    /// After waking up the sensor you should wait about 30s before reading the sensor data to wait
    /// for the sensor to stabilize
    pub fn set_sleeping(
        &mut self,
        sleeping: Sleep,
    ) -> nb::Result<(), <Serial as Write<u8>>::Error> {
        self.send_command(Command::SetSleep, sleeping as u16, sleeping == Sleep::Sleep)
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[repr(u16)]
pub enum Sleep {
    Sleep = 0,
    Wakeup = 1,
}

impl<Error, Serial: Read<u8, Error = Error> + Write<u8, Error = Error>>
    Pms700X<Error, Serial, Active>
{
    pub fn read(&mut self) -> nb::Result<SensorData, Error> {
        self.reader.fill_data(&mut self.serial)?;
        if self.reader.validate_data() {
            Ok(SensorData::from_raw(&self.reader.data))
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<Error, Serial: Read<u8, Error = Error> + Write<u8, Error = Error>>
    Pms700X<Error, Serial, Passive>
{
    pub fn read(&mut self) -> nb::Result<SensorData, Error> {
        self.send_command(Command::ReadPassive, 0, true)?;
        Ok(SensorData::from_raw(&self.reader.data))
    }
}

impl SensorReader {
    fn fill_data<Serial: Read<u8>>(
        &mut self,
        serial: &mut Serial,
    ) -> nb::Result<(), Serial::Error> {
        let byte = serial.read()?;
        let offset = self.byte_offset;
        self.byte_offset += 1;
        self.data[offset as usize] = byte;
        match (offset, byte) {
            (0, START_HEADER_1) => Err(nb::Error::WouldBlock),
            (0, _) => {
                // wait until we find the start header
                self.byte_offset = 0;
                Err(nb::Error::WouldBlock)
            }
            (1, START_HEADER_2) => Err(nb::Error::WouldBlock),
            (2, 0) => Err(nb::Error::WouldBlock),
            (2, _) => {
                // we only allow length <= 32
                self.byte_offset = 0;
                Err(nb::Error::WouldBlock)
            }
            (3, length_low_byte) => {
                self.length = length_low_byte;
                if self.length > MAX_RESPONSE_SIZE as u8 {
                    self.byte_offset = 0;
                }
                Err(nb::Error::WouldBlock)
            }
            (offset, _) => {
                if offset >= self.length + 3 {
                    self.byte_offset = 0;
                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }
    }

    fn validate_data(&self) -> bool {
        let mut sum = 0;
        for byte in self.data.iter().take(self.length as usize + 2) {
            sum += *byte as u16;
        }

        let checksum = u16::from_be_bytes([
            self.data[self.length as usize + 2],
            self.data[self.length as usize + 3],
        ]);
        checksum == sum
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
enum Command {
    None = 0,
    ReadPassive = 0xe2,
    SetMode = 0xe1,
    SetSleep = 0xe4,
}

impl Default for Command {
    fn default() -> Self {
        Command::None
    }
}

#[derive(Default)]
pub struct CommandWriter {
    command: Command,
    data_high: u8,
    data_low: u8,
    state: u8,
    verify_high: u8,
    verify_low: u8,
}

impl CommandWriter {
    fn new(command: Command, data: u16) -> Self {
        let data_bytes: [u8; 2] = data.to_le_bytes();
        let verify = START_HEADER_1 as u16
            + START_HEADER_2 as u16
            + command as u8 as u16
            + data_bytes[0] as u16
            + data_bytes[1] as u16;
        let verify_bytes: [u8; 2] = verify.to_le_bytes();
        CommandWriter {
            command,
            data_high: data_bytes[1],
            data_low: data_bytes[0],
            state: 0,
            verify_high: verify_bytes[1],
            verify_low: verify_bytes[0],
        }
    }

    fn write<Serial: Write<u8>>(&mut self, serial: &mut Serial) -> nb::Result<(), Serial::Error> {
        if self.state < 7 {
            let write_byte = match self.state {
                0 => START_HEADER_1,
                1 => START_HEADER_2,
                2 => self.command as u8,
                3 => self.data_high,
                4 => self.data_low,
                5 => self.verify_high,
                6 => self.verify_low,
                _ => unreachable!(),
            };
            serial.write(write_byte)?;

            self.state += 1;
        }

        if self.state == 7 {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
