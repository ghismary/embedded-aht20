#![doc = include_str!("../README.md")]
#![deny(unsafe_code, missing_docs)]
#![no_std]

use bitflags::bitflags;
use crc::{Crc, CRC_8_NRSC_5};

#[cfg(not(feature = "async"))]
use embedded_hal as hal;
#[cfg(feature = "async")]
use embedded_hal_async as hal;

use hal::delay::DelayNs;
use hal::i2c::{I2c, SevenBitAddress};

use weather_utils::{unit::Celcius, TemperatureAndRelativeHumidity};

/// The default I2C address.
pub const DEFAULT_I2C_ADDRESS: SevenBitAddress = 0x38;

const CHECK_STATUS_COMMAND: &[u8] = &[0b0111_0001];
const INITIALIZATION_COMMAND: &[u8] = &[0b1011_1110, 0x08, 0x00];
const TRIGGER_MEASUREMENT_COMMAND: &[u8] = &[0b1010_1100, 0x33, 0x00];
const SOFT_RESET_COMMAND: &[u8] = &[0b1011_1010];

/// All possible errors generated when using the Aht20 struct.
#[derive(Debug)]
pub enum Error<I2cError>
where
    I2cError: hal::i2c::Error,
{
    /// I²C bus error.
    I2c(I2cError),
    /// The computed CRC and the one sent by the device mismatch.
    InvalidCrc,
    /// The device is busy at a time where it was not expected to.
    UnexpectedBusy,
}

impl<I2cError> From<I2cError> for Error<I2cError>
where
    I2cError: hal::i2c::Error,
{
    fn from(value: I2cError) -> Self {
        Error::I2c(value)
    }
}

#[derive(Debug)]
struct SensorMeasurement {
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
    /// The measured relative humidity (in %).
    pub fn humidity(&self) -> f32 {
        ((self.raw_humidity as f32) / ((1 << 20) as f32)) * 100.0
    }

    /// The measured temperature (in °C).
    pub fn temperature(&self) -> f32 {
        ((self.raw_temperature as f32) / ((1 << 20) as f32)) * 200.0 - 50.0
    }
}

impl From<SensorMeasurement> for TemperatureAndRelativeHumidity<Celcius> {
    fn from(value: SensorMeasurement) -> Self {
        TemperatureAndRelativeHumidity::<Celcius>::new(value.temperature(), value.humidity())
    }
}

bitflags! {
    struct SensorStatus: u8 {
        const BUSY = 0b1000_0000;
        const CALIBRATED = 0b0000_1000;
    }
}

impl SensorStatus {
    fn is_calibrated(&self) -> bool {
        self.contains(SensorStatus::CALIBRATED)
    }

    fn is_ready(&self) -> bool {
        !self.contains(SensorStatus::BUSY)
    }
}

/// AHT20 device driver.
#[derive(Debug)]
pub struct Aht20<I2C, D> {
    i2c: I2C,
    address: SevenBitAddress,
    delay: D,
}

impl<I2C, D> Aht20<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    /// Create a new instance of the AHT20 device.
    #[maybe_async_cfg::maybe(
        sync(not(feature = "async"), keep_self),
        async(feature = "async", keep_self)
    )]
    pub async fn new(
        i2c: I2C,
        address: SevenBitAddress,
        delay: D,
    ) -> Result<Self, Error<I2C::Error>> {
        let mut dev = Self {
            i2c,
            address,
            delay,
        };

        while !dev.check_status().await?.is_calibrated() {
            dev.send_initialize().await?;
            dev.delay_ms(10).await;
        }

        Ok(dev)
    }

    /// Perform a measurement.
    ///
    /// The measurement takes at least 80 ms to be performed.
    #[maybe_async_cfg::maybe(
        sync(not(feature = "async"), keep_self),
        async(feature = "async", keep_self)
    )]
    pub async fn measure(
        &mut self,
    ) -> Result<TemperatureAndRelativeHumidity<Celcius>, Error<I2C::Error>> {
        self.send_trigger_measurement().await?;

        // Wait for measurement to be ready
        self.delay_ms(80).await;
        while !self.check_status().await?.is_ready() {
            self.delay_ms(1).await;
        }

        let mut buffer = [0u8; 7];
        self.i2c
            .read(self.address, &mut buffer)
            .await
            .map_err(Error::I2c)?;

        let data = &buffer[..6];
        let crc = buffer[6];
        self.check_crc(data, crc)?;

        let status = SensorStatus::from_bits_retain(buffer[0]);
        if !status.is_ready() {
            return Err(Error::UnexpectedBusy);
        }

        let measurement = SensorMeasurement::from(&data[1..6]);
        Ok(measurement.into())
    }

    /// Perform a soft reset to force the device into a well-defined state
    /// without removing the power supply.
    #[maybe_async_cfg::maybe(
        sync(not(feature = "async"), keep_self),
        async(feature = "async", keep_self)
    )]
    pub async fn soft_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, SOFT_RESET_COMMAND)
            .await
            .map_err(Error::I2c)?;
        self.delay_ms(20).await;
        Ok(())
    }

    fn check_crc(&self, data: &[u8], crc_value: u8) -> Result<(), Error<I2C::Error>> {
        let crc = Crc::<u8>::new(&CRC_8_NRSC_5);
        let mut digest = crc.digest();
        digest.update(data);
        if digest.finalize() != crc_value {
            return Err(Error::InvalidCrc);
        }
        Ok(())
    }

    #[maybe_async_cfg::maybe(
        sync(not(feature = "async"), keep_self),
        async(feature = "async", keep_self)
    )]
    async fn check_status(&mut self) -> Result<SensorStatus, Error<I2C::Error>> {
        let mut buffer = [0];
        self.i2c
            .write_read(self.address, CHECK_STATUS_COMMAND, &mut buffer)
            .await
            .map_err(Error::I2c)?;
        Ok(SensorStatus::from_bits_retain(buffer[0]))
    }

    #[maybe_async_cfg::maybe(
        sync(not(feature = "async"), keep_self),
        async(feature = "async", keep_self)
    )]
    async fn delay_ms(&mut self, duration: u32) {
        self.delay.delay_ms(duration).await;
    }

    #[maybe_async_cfg::maybe(
        sync(not(feature = "async"), keep_self),
        async(feature = "async", keep_self)
    )]
    async fn send_initialize(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, INITIALIZATION_COMMAND)
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    #[maybe_async_cfg::maybe(
        sync(not(feature = "async"), keep_self),
        async(feature = "async", keep_self)
    )]
    async fn send_trigger_measurement(&mut self) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, TRIGGER_MEASUREMENT_COMMAND)
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::*;
    use embedded_hal_mock::eh1::delay::StdSleep as Delay;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    #[test]
    fn measure() {
        let expectations = [
            I2cTransaction::write_read(
                DEFAULT_I2C_ADDRESS,
                CHECK_STATUS_COMMAND.to_vec(),
                [SensorStatus::CALIBRATED.bits()].to_vec(),
            ),
            I2cTransaction::write(DEFAULT_I2C_ADDRESS, TRIGGER_MEASUREMENT_COMMAND.to_vec()),
            I2cTransaction::write_read(
                DEFAULT_I2C_ADDRESS,
                CHECK_STATUS_COMMAND.to_vec(),
                [SensorStatus::CALIBRATED.bits()].to_vec(),
            ),
            I2cTransaction::read(
                DEFAULT_I2C_ADDRESS,
                [0b0000_1000, 0x7b, 0xb3, 0x05, 0x9d, 0x49, 0x7d].to_vec(),
            ),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut device = Aht20::new(&mut i2c, DEFAULT_I2C_ADDRESS, Delay {}).unwrap();
        let measurement = device.measure().unwrap();
        assert!((measurement.temperature.celcius() - 20.18).abs() < 0.01);
        assert!((measurement.relative_humidity - 48.32).abs() < 0.01);
        i2c.done();
    }

    #[test]
    fn soft_reset() {
        let expectations = [
            I2cTransaction::write_read(
                DEFAULT_I2C_ADDRESS,
                CHECK_STATUS_COMMAND.to_vec(),
                [SensorStatus::CALIBRATED.bits()].to_vec(),
            ),
            I2cTransaction::write(DEFAULT_I2C_ADDRESS, SOFT_RESET_COMMAND.to_vec()),
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut device = Aht20::new(&mut i2c, DEFAULT_I2C_ADDRESS, Delay {}).unwrap();
        device.soft_reset().unwrap();
        i2c.done();
    }
}
