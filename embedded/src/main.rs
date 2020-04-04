#![no_main]
#![no_std]

extern crate panic_halt;

use core::{cell::RefCell, f32::consts::PI, ops::DerefMut};

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;

use stm32f4xx_hal as hal;

use hal::{
    adc::{config::AdcConfig, Adc},
    delay::Delay,
    gpio::gpioa::{PA0, PA6, PA7},
    gpio::gpiob::{PB8, PB9},
    gpio::{AlternateOD, Analog, AF4},
    i2c::I2c,
    interrupt,
    prelude::*,
    serial::Serial,
    stm32::{self, CorePeripherals, Peripherals, ADC1, ADC2, ADC3, I2C1, USART2},
    timer::Timer,
};

use embedded::handler;

type I2cBus = I2C1;
type I2cPin = (PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>);
type USBTx = hal::serial::Tx<USART2>;

// shared items
static UART_TX: Mutex<RefCell<Option<USBTx>>> = Mutex::new(RefCell::new(None));

static TIMER: Mutex<RefCell<Option<Timer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));

struct Devices {
    imu: handler::bmx055::IMU<I2cBus, I2cPin>,
    elbow: handler::potentio::Potentiometer<PA0<Analog>>,
}

static DEVICES: Mutex<RefCell<Option<Devices>>> = Mutex::new(RefCell::new(None));

// const parameters
const CLOCK: u32 = 100; // Hertz
const INIT_COUNT_IMU: u32 = 1000;
const INIT_COUNT_ADC: u32 = 100;
const HEADER: [u8; 2] = [0xE0, 0xE0];
const POTENTIO_GAIN: f32 = 1.0 / 14.0;
const DEG_TO_RAD: f32 = PI / 180.0;

#[entry]
fn main() -> ! {
    if let (Some(peripherals), Some(core_peripherals)) =
        (Peripherals::take(), CorePeripherals::take())
    {
        // clock
        let rcc = peripherals.RCC.constrain();
        let clock = rcc
            .cfgr
            .use_hse(8.mhz())
            .hclk(180.mhz())
            .pclk1(45.mhz())
            .pclk2(90.mhz())
            .sysclk(180.mhz())
            .freeze();

        let mut delay = Delay::new(core_peripherals.SYST, clock);

        // usart
        let gpioa = peripherals.GPIOA.split();
        let gpio_tx = gpioa.pa2.into_alternate_af7();
        let gpio_rx = gpioa.pa3.into_alternate_af7();

        let usart = Serial::usart2(
            peripherals.USART2,
            (gpio_tx, gpio_rx),
            hal::serial::config::Config::default().baudrate(hal::time::Bps(115200)),
            clock,
        )
        .unwrap();

        let (tx, _rx) = usart.split();

        // LED
        let mut green_led = gpioa.pa5.into_push_pull_output();

        // switch
        let switch = gpioa.pa10.into_pull_down_input();

        // i2c
        let gpiob = peripherals.GPIOB.split();
        let i2c_scl = gpiob.pb8.into_alternate_af4_open_drain();
        let i2c_sda = gpiob.pb9.into_alternate_af4_open_drain();

        let i2c = I2c::i2c1(
            peripherals.I2C1,
            (i2c_scl, i2c_sda),
            hal::time::KiloHertz(100),
            clock,
        );

        // adc
        let mut elbow_adc = Adc::adc3(peripherals.ADC3, true, AdcConfig::default());
        let mut elbow_potentio = gpioa.pa0.into_analog();

        // sensor
        let mut bmx055 = handler::bmx055::IMU::new(i2c, 0.1, CLOCK as f32);
        let mut elbow = handler::potentio::Potentiometer::new(elbow_adc, elbow_potentio);

        // initialize
        green_led.set_low().unwrap();
        while switch.is_low().unwrap() {
            delay.delay_ms(10u8);
        }
        green_led.set_high().unwrap();
        bmx055.initialize(&mut delay, 10, INIT_COUNT_IMU);
        elbow.initialize(&mut delay, 10, INIT_COUNT_ADC);

        green_led.set_low().unwrap();

        // interrupt
        let mut timer_interrupt = Timer::tim2(peripherals.TIM2, hal::time::Hertz(CLOCK), clock);

        timer_interrupt.listen(hal::timer::Event::TimeOut);

        cortex_m::interrupt::free(|cs| {
            *UART_TX.borrow(cs).borrow_mut() = Some(tx);
            *TIMER.borrow(cs).borrow_mut() = Some(timer_interrupt);
            *DEVICES.borrow(cs).borrow_mut() = Some(Devices { imu: bmx055, elbow });
        });

        unsafe {
            hal::stm32::NVIC::unmask(hal::stm32::Interrupt::TIM2);
        };

        loop {
            green_led.toggle().unwrap();
            delay.delay_ms(250u8);
        }
    }

    loop {}
}

#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut timer) = TIMER.borrow(cs).borrow_mut().deref_mut() {
            timer.clear_interrupt(hal::timer::Event::TimeOut);

            if let Some(ref mut tx) = UART_TX.borrow(cs).borrow_mut().deref_mut() {
                if let Some(ref mut dev) = DEVICES.borrow(cs).borrow_mut().deref_mut() {
                    dev.imu.update();
                    let angle = match dev.elbow.read_rad() {
                        Ok(val) => val,
                        Err(_) => 0.0_f32,
                    };
                    handler::serial::transmit_quaternion(tx, dev.imu.imu_data, &HEADER);
                    handler::serial::transmit_base(tx, &angle.to_le_bytes());
                }
            }
        }
    });
}
