#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    gpio::Io,
    i2c::master::{Config, I2c},
    time::Rate,
};
use esp_println as _;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");

    // I2C setup
    let io = Io::new(peripherals.IO_MUX);
    
    let i2c_config = Config::default()
        .with_frequency(Rate::from_khz(100));

    let mut i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

    const BMI160_ADDR: u8 = 0x69;
    const CHIP_ID_REG: u8 = 0x00;
    const CMD_REG: u8 = 0x7E;
    const ACCEL_DATA_REG: u8 = 0x12;
    const GYRO_DATA_REG: u8 = 0x0C;

    const SOFT_RESET_CMD: u8 = 0xB6;
    const ACCEL_NORMAL_MODE: u8 = 0x11;
    const GYRO_NORMAL_MODE: u8 = 0x15;

    info!("Initializing BMI160...");

    let _ = i2c.write(BMI160_ADDR, &[CMD_REG, SOFT_RESET_CMD]);
    Timer::after(Duration::from_millis(10)).await;

    // Waking up acceleromteer
    let _ = i2c.write(BMI160_ADDR, &[CMD_REG, ACCEL_NORMAL_MODE]);
    Timer::after(Duration::from_millis(4)).await;

   // Waking up gyroscope  
    let _ = i2c.write(BMI160_ADDR, &[CMD_REG, GYRO_NORMAL_MODE]);
    Timer::after(Duration::from_millis(80)).await;

    // Verifying chip id after initalization
    let mut chip_id = [0u8; 1];
    if i2c.write_read(BMI160_ADDR, &[CHIP_ID_REG], &mut chip_id).is_ok() {
        info!("BMI160 initialized! Chip ID: 0x{:02X}", chip_id[0]);
    }

    info!("Starting data collection...");

    loop {
        let mut accel_raw = [0u8; 6]; 
        if i2c.write_read(BMI160_ADDR, &[ACCEL_DATA_REG], &mut accel_raw).is_ok() {
            let ax = i16::from_le_bytes([accel_raw[0], accel_raw[1]]);
            let ay = i16::from_le_bytes([accel_raw[2], accel_raw[3]]);
            let az = i16::from_le_bytes([accel_raw[4], accel_raw[5]]);
            info!("Accel: X={}, Y={}, Z={}", ax, ay, az);
        }

        let mut gyro_raw = [0u8; 6];
        if i2c.write_read(BMI160_ADDR, &[GYRO_DATA_REG], &mut gyro_raw).is_ok() {
            let gx = i16::from_le_bytes([gyro_raw[0], gyro_raw[1]]);
            let gy = i16::from_le_bytes([gyro_raw[2], gyro_raw[3]]);
            let gz = i16::from_le_bytes([gyro_raw[4], gyro_raw[5]]);
            info!("Gyro: X={}, Y={}, Z={}", gx, gy, gz);
        }

        Timer::after(Duration::from_millis(500)).await;
    }
}
