#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use cortex_m_rt::entry;
use smart_leds_trait::{SmartLedsWrite, RGB8};
use embassy_stm32::timer::{ Channel, CountingMode::{EdgeAlignedUp} };
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::time::hz;
use embassy_stm32::gpio::OutputType::{ PushPull, OpenDrain };
use embassy_time::Delay;
use embedded_hal::blocking::delay::DelayMs;

use {defmt_rtt as _, panic_probe as _};

//use pwm_dma::Ws2812Pwm; 

pub const LED_COUNT: usize = 8;
pub const RESET: usize = 40;


#[entry]
fn main() -> ! {
    
    let dp = embassy_stm32::init(Default::default());

    
    /// buzzer tests
    let buzz_pin = PwmPin::new_ch1(dp.PB6, PushPull);
    let mut buzz_pwm = SimplePwm::new(dp.TIM4, Some(buzz_pin), None, None, None, hz(2000), EdgeAlignedUp);
    let buzz_duty = buzz_pwm.get_max_duty();
    buzz_pwm.set_duty(Channel::Ch1, buzz_duty / 2);
    buzz_pwm.enable(Channel::Ch1);
    
    
    /// simplepwm for led
//    let led_pin = PwmPin::new_ch2(dp.PB5, PushPull); // this should be  into alternate or drain
//    let mut led_pwm = SimplePwm::new(dp.TIM3, None, Some(led_pin), None, None, hz(26000000), EdgeAlignedUp);
//    let buzz_duty = led_pwm.get_max_duty();
//    led_pwm.set_duty(Channel::Ch2, buzz_duty / 2);
//    led_pwm.enable(Channel::Ch2);
//    
//    let led_buf = {
//        static mut LED_BUF: [u16; 24 * (LED_COUNT) + RESET] = [0; 24 * (LED_COUNT) + RESET];
//        unsafe { &mut LED_BUF }
//    };
//    
//    let buf = [
//        RGB8::new(255, 0, 0),
//        RGB8::new(0, 0, 0),
//        RGB8::new(0, 0, 0),
//        RGB8::new(255, 30, 0),
//        RGB8::new(255, 180, 0),
//        RGB8::new(0, 0, 0),
//        RGB8::new(0, 0, 0),
//        RGB8::new(0, 255, 0),
//        ];
    
    
    let wait = 300_u32;

    loop {
        
        /// buzzer
        buzz_pwm.set_frequency(hz(200));
        buzz_pwm.enable(Channel::Ch1);
        Delay.delay_ms(wait);
        buzz_pwm.disable(Channel::Ch1);
        Delay.delay_ms(wait);
        
        
        // led
//        ws.write((0..LED_COUNT).enumerate().map(|(ix, _)| buf[ix]))
//        .unwrap();
//        delay.delay_ms(44_u32);
    }
}


