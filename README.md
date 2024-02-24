[![crates.io](https://img.shields.io/crates/v/embedded-aht20.svg)](https://crates.io/crates/embedded-aht20)
[![License](https://img.shields.io/crates/l/embedded-aht20.svg)](https://crates.io/crates/embedded-aht20)
[![Documentation](https://docs.rs/embedded-aht20/badge.svg)](https://docs.rs/embedded-aht20)

# embedded-aht20

This is a platform agnostic Rust driver the AHT20 digital humidity and
temperature sensors using the [`embedded-hal`] and [`embedded-hal-async`] traits.

[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
[`embedded-hal-async`]: https://github.com/rust-embedded/embedded-hal

## The device

The AHT20 sensor is a humidity and temperature sensor, that is fully
calibrated, has an excellent long-term stability, and is using an I²C
interface.

Its typical application scope includes: HVAC system, dehumidifier, test
and inspection equipment, consumer goods, automobiles, automatic control, data
recorder, weather station, household appliances, humidity regulation, medical
and other related temperature and humidity detection and control.

It has a typical accuracy of ±0.3 °C for the temperature, and of ±2 % for the
relative humidity.

Here are its electrical specifications:

| Parameter         | Condition | Min | Typical | Max  | Unit |
| ----------------- | --------- | --- | ------- | ---- | ---- |
| Voltage           | Typical   | 2.0 | 3.3     | 5.5  | V    |
| Current           | Dormant   |     |         | 0.25 | µA   |
| Current           | Measure   |     | 23      |      | µA   |
| Power consumption | Dormant   |     |         | 0.9  | µW   |
| Power consumption | Measure   |     | 0.07    |      | mW   |
| Power consumption | Average   |     | 3.3     |      | µW   |

### Documentation:

- [Datasheet](http://www.aosong.com/userfiles/files/media/Data%20Sheet%20AHT20.pdf)
- [Arduino driver](https://github.com/dvarrel/AHT20)

## Features

- [x] Perform a measurement of temperature and relative humidity.
- [x] Do a sofware set.
- [ ] Include a no floating-point variant for systems without fpu.

## Additional features

This crate relies on the [`weather-utils`] crate. When you perform a
temperature and humidity measurement, the result is a struct from the
[`weather-utils`] crate, giving you the features provided by this crate for free.
For example, you can immediately compute the absolute humidity value (in g/m³)
from the result of [`Aht20::measure`]. If you are also using an other sensor,
such as the [`QMP6988`], to get the barometric pressure, you can also compute
the altitude.

[`weather-utils`]: https://crates.io/crates/weather-utils
[`QMP6988`]: https://crates.io/crates/embedded-qmp6988

## Usage

To use this driver, import what you need from this crate and an `embedded-hal`
implentation, then instatiate the device.

```rust,no_run
use embedded_aht20::{Aht20, DEFAULT_I2C_ADDRESS};
use linux_embedded_hal as hal;

fn main() -> Result<(), embedded_aht20::Error<hal::I2CError>> {
    // Create the I2C device from the chosen embedded-hal implementation,
    // in this case linux-embedded-hal
    let mut i2c = match hal::I2cdev::new("/dev/i2c-1") {
        Err(err) => {
            eprintln!("Could not create I2C device: {}", err);
            std::process::exit(1);
        }
        Ok(i2c) => i2c,
    };
    if let Err(err) = i2c.set_slave_address(DEFAULT_I2C_ADDRESS as u16) {
        eprintln!("Could not set I2C slave address: {}", err);
        std::process::exit(1);
    }

    // Create the sensor
    let mut sensor = Aht20::new(i2c, DEFAULT_I2C_ADDRESS, hal::Delay {})?;

    // Perform a temperature and humidity measurement
    let measurement = sensor.measure()?;
    println!(
        "Temperature: {:.2} °C, Relative humidity: {:.2} %",
        measurement.temperature.celcius(),
        measurement.relative_humidity
    );
    Ok(())
}
```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
