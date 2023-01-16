#![no_std]

use crc_any::CRCu8;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Write, WriteRead};

pub const AHT20_I2C_ADDRESS: u8 = 0b0011_1000; // 0x38;

enum Command {
    CheckStatus = 0b0111_0001,
    Initialization = 0b1011_1110,
    TriggerMeasurement = 0b1010_1100,
    SoftReset = 0b1011_1010,
}

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    InvalidCrc,
    UnexpectedBusy,
}

#[derive(Debug)]
pub struct SensorMeasurement {
    raw_humidity: u32,
    raw_temperature: u32,
}

impl From<&[u8]> for SensorMeasurement {
    fn from(data: &[u8]) -> Self {
        let raw_humidity: u32 =
            ((data[0] as u32) << 12) | ((data[1] as u32) << 4) | ((data[2] >> 4) as u32);
        let raw_temperature: u32 =
            (((data[2] & 0b0000_1111) as u32) << 16) | ((data[3] as u32) << 8) | (data[4] as u32);
        SensorMeasurement {
            raw_humidity,
            raw_temperature,
        }
    }
}

impl SensorMeasurement {
    #[cfg(feature = "floating-point")]
    pub fn humidity(&self) -> f32 {
        ((self.raw_humidity as f32) / ((1 << 20) as f32)) * 100.0
    }

    #[cfg(feature = "floating-point")]
    pub fn temperature(&self) -> f32 {
        ((self.raw_temperature as f32) / ((1 << 20) as f32)) * 200.0 - 50.0
    }

    #[cfg(not(feature = "floating-point"))]
    pub fn humidity(&self) -> (i8, i8) {
        let percent = self.raw_humidity * 100;
        let int_part = (percent >> 20) as i8;
        let frac_part = (((percent % (1 << 20)) * 100) >> 20) as i8;
        (int_part, frac_part)
    }

    #[cfg(not(feature = "floating-point"))]
    pub fn temperature(&self) -> (i8, i8) {
        let real = self.raw_temperature * 200;
        let int_part = ((real >> 20) - 50) as i8;
        let frac_part = (((real % (1 << 20)) * 100) >> 20) as i8;
        (int_part, frac_part)
    }
}

enum SensorStatusBits {
    Busy = 0b1000_0000,
    Calibrated = 0b0000_1000,
}

struct SensorStatus(u8);

impl SensorStatus {
    pub fn new(status: u8) -> Self {
        Self(status)
    }

    pub fn is_calibrated(&self) -> bool {
        (self.0 & SensorStatusBits::Calibrated as u8) != 0
    }

    pub fn is_ready(&self) -> bool {
        (self.0 & SensorStatusBits::Busy as u8) == 0
    }
}

#[derive(Debug)]
pub struct AHT20<I2C, D> {
    i2c: I2C,
    address: u8,
    delay: D,
}

impl<I2C, D, E> AHT20<I2C, D>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    D: DelayMs<u16>,
{
    pub fn new(i2c: I2C, address: u8, delay: D) -> Result<Self, Error<E>> {
        let mut dev = Self {
            i2c,
            address,
            delay,
        };

        dev.delay_ms(40_u16);

        while !dev.check_status()?.is_calibrated() {
            dev.send_initialize()?;
            dev.delay_ms(10_u16);
        }

        Ok(dev)
    }

    pub fn measure(&mut self) -> Result<SensorMeasurement, Error<E>> {
        self.send_trigger_measurement()?;

        // Wait for measurement to be ready
        self.delay_ms(80_u16);
        while !self.check_status()?.is_ready() {
            self.delay_ms(1_u16);
        }

        let mut buffer = [0u8; 7];
        self.i2c
            .write_read(self.address, &[0u8], &mut buffer)
            .map_err(Error::I2c)?;

        let data = &buffer[..6];
        let crc = buffer[6];
        self.check_crc(data, crc)?;

        let status = SensorStatus::new(buffer[0]);
        if !status.is_ready() {
            return Err(Error::UnexpectedBusy);
        }

        Ok(SensorMeasurement::from(&data[1..6]))
    }

    pub fn soft_reset(&mut self) -> Result<(), Error<E>> {
        let command = [Command::SoftReset as u8];
        self.i2c.write(self.address, &command).map_err(Error::I2c)?;
        self.delay_ms(20u16);
        Ok(())
    }

    fn check_crc(&self, data: &[u8], crc_value: u8) -> Result<(), Error<E>> {
        let mut crc = CRCu8::create_crc(0x31, 8, 0xff, 0x00, false);
        crc.digest(data);
        if crc.get_crc() != crc_value {
            return Err(Error::InvalidCrc);
        }
        Ok(())
    }

    fn check_status(&mut self) -> Result<SensorStatus, Error<E>> {
        let command = [Command::CheckStatus as u8];
        let mut buffer = [0_u8; 1];
        self.i2c
            .write_read(self.address, &command, &mut buffer)
            .map_err(Error::I2c)?;
        Ok(SensorStatus(buffer[0]))
    }

    fn delay_ms(&mut self, duration: u16) {
        self.delay.delay_ms(duration);
    }

    fn send_initialize(&mut self) -> Result<(), Error<E>> {
        let command = [Command::Initialization as u8, 0b0000_1000, 0b0000_0000];
        self.i2c.write(self.address, &command).map_err(Error::I2c)?;
        Ok(())
    }

    fn send_trigger_measurement(&mut self) -> Result<(), Error<E>> {
        let command = [Command::TriggerMeasurement as u8, 0b0011_0011, 0b0000_0000];
        self.i2c.write(self.address, &command).map_err(Error::I2c)?;
        Ok(())
    }
}
