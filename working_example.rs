#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use bmi160_esp32_minimal::{Bmi160, Bmi160Error};
use esp_hal::{
    gpio::Io,
    i2c::master::{Config, I2c},
    time::Rate,
};

use defmt_rtt as _; // This line fixes the linker error!
use esp_println::println as info;
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;


#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);


    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");


    let io = Io::new(peripherals.IO_MUX);
    
    let i2c_config = Config::default()
        .with_frequency(Rate::from_khz(100));
    esp_alloc::heap_allocator!(size: 16 * 1024);
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

    let mut bmi160 = Bmi160::new(i2c, 0x69); // 0x68 if your SA0 pin is low
    
    info!("BMI160 created, initializing...");

    match bmi160.init().await {
        Ok(()) => {
            info!("BMI160 initialized successfully!");
        },
        Err(Bmi160Error::InvalidChipId(id)) => {
            info!("Wrong chip ID: 0x{:02X} (expected 0xD1)", id);
            info!("Check wiring and I2C address (try 0x68 instead of 0x69)");
            return;
        },
        Err(e) => {
            info!("BMI160 initialization failed: ", );
            info!("Check wiring: SDA=GPIO21, SCL=GPIO22, VCC=3.3V, GND=GND");
            return;
        }
    }

    loop {
        match bmi160.read_all() {
            Ok(data) => {
                info!("Raw - Accel: X={}, Y={}, Z={}", 
                    data.accel.x, data.accel.y, data.accel.z);
                info!("Raw - Gyro: X={}, Y={}, Z={}", 
                    data.gyro.x, data.gyro.y, data.gyro.z);
                
                let (ax, ay, az) = data.accel.to_g();
                let (gx, gy, gz) = data.gyro.to_dps();
                
                info!("Physical - Accel: {}g, {}g, {}g", ax, ay, az);
                info!("Physical - Gyro: {}°/s, {}°/s, {}°/s", gx, gy, gz);
            }
            Err(e) => {
                info!("Read error: ");
            }
        }
        
        Timer::after(Duration::from_millis(500)).await;
    }
}
