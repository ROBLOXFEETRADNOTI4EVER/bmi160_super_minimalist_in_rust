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

// Data structures
#[derive(Debug, Clone, Copy)]
pub struct AccelData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[derive(Debug, Clone, Copy)]
pub struct GyroData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[derive(Debug, Clone, Copy)]
pub struct SensorData {
    pub accel: AccelData,
    pub gyro: GyroData,
}

// BMI160 Driver Struct - FIXED: Use concrete type
pub struct Bmi160 {
    i2c: I2c<'static, esp_hal::Blocking>,
    address: u8,
}

impl Bmi160 {
    // Constants
    const CHIP_ID_REG: u8 = 0x00;
    const CMD_REG: u8 = 0x7E;
    const ACCEL_DATA_REG: u8 = 0x12;
    const GYRO_DATA_REG: u8 = 0x0C;
    
    const SOFT_RESET_CMD: u8 = 0xB6;
    const ACCEL_NORMAL_MODE: u8 = 0x11;
    const GYRO_NORMAL_MODE: u8 = 0x15;
    
    const EXPECTED_CHIP_ID: u8 = 0xD1;

    /// Create new BMI160 instance
    pub fn new(i2c: I2c<'static, esp_hal::Blocking>, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Initialize the BMI160 sensor
    pub async fn init(&mut self) -> Option<()> {
        info!("Initializing BMI160...");

        // Soft reset
        self.write_register(Self::CMD_REG, Self::SOFT_RESET_CMD)?;
        Timer::after(Duration::from_millis(10)).await;

        // Wake up accelerometer
        self.write_register(Self::CMD_REG, Self::ACCEL_NORMAL_MODE)?;
        Timer::after(Duration::from_millis(4)).await;

        // Wake up gyroscope
        self.write_register(Self::CMD_REG, Self::GYRO_NORMAL_MODE)?;
        Timer::after(Duration::from_millis(80)).await;

        let chip_id = self.read_chip_id()?;
        if chip_id == Self::EXPECTED_CHIP_ID {
            info!("BMI160 initialized! Chip ID: 0x{:02X}", chip_id);
            Some(())
        } else {
            info!("BMI160 init failed! Wrong chip ID: 0x{:02X}", chip_id);
            None
        }
    }

    pub fn read_chip_id(&mut self) -> Option<u8> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.address, &[Self::CHIP_ID_REG], &mut buffer)
            .ok()?;
        Some(buffer[0])
    }

    pub fn read_accel(&mut self) -> Option<AccelData> {
        let mut buffer = [0u8; 6];
        self.i2c
            .write_read(self.address, &[Self::ACCEL_DATA_REG], &mut buffer)
            .ok()?;

        let x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let z = i16::from_le_bytes([buffer[4], buffer[5]]);

        Some(AccelData { x, y, z })
    }

    pub fn read_gyro(&mut self) -> Option<GyroData> {
        let mut buffer = [0u8; 6];
        self.i2c
            .write_read(self.address, &[Self::GYRO_DATA_REG], &mut buffer)
            .ok()?;

        let x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let z = i16::from_le_bytes([buffer[4], buffer[5]]);

        Some(GyroData { x, y, z })
    }

    pub fn read_all(&mut self) -> Option<SensorData> {
        let accel = self.read_accel()?;
        let gyro = self.read_gyro()?;
        Some(SensorData { accel, gyro })
    }

    fn write_register(&mut self, reg: u8, value: u8) -> Option<()> {
        self.i2c.write(self.address, &[reg, value]).ok()
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");

    let io = Io::new(peripherals.IO_MUX);
    
    let i2c_config = Config::default()
        .with_frequency(Rate::from_khz(100));

    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

    let mut bmi160 = Bmi160::new(i2c, 0x69);

    if bmi160.init().await.is_some() {
        info!("Starting data collection...");
        
        loop {
            if let Some(data) = bmi160.read_all() {
                info!("Accel: X={}, Y={}, Z={}", data.accel.x, data.accel.y, data.accel.z);
                info!("Gyro: X={}, Y={}, Z={}", data.gyro.x, data.gyro.y, data.gyro.z);
            } else {
                info!("Failed to read sensor data");
            }

            Timer::after(Duration::from_millis(500)).await;
        }
    } else {
        info!("BMI160 initialization failed!");
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
