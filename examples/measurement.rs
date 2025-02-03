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
        measurement.temperature.celsius(),
        measurement.relative_humidity
    );
    Ok(())
}
