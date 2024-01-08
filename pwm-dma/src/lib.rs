#![no_std]

use bitvec::prelude::{BitArray, Msb0};
use core::mem::{self, size_of};
use embedded_dma::Word;
use embedded_hal::PwmPin;
use fugit::RateExtU32;
use smart_leds_trait::{SmartLedsWrite, RGB8};


/// tim_chan use needs to be changed; disable & enable dma funcs arent used anymore, find replacement through embassy
///embassy dma already has a transfer func


//use stm32f4xx_hal::{
//    dma::traits::Channel, /// inside timer
//    dma::traits::DMASet, /// inside timer??
//    dma::traits::PeriAddress, /// inside _generated::peripherals
//    dma::traits::Stream, /// inside timer
//    dma::ChannelX, /// inside timer
//    dma::MemoryToPeripheral, /// inside _generated::peripherals
//    rcc::Clocks,  /// inside timer??
//    timer::{Ch, Pins, PwmExt, CCR}, /// inside simple pwm & tiimer??
//};
//use stm32f4xx_hal::timer::PwmHz; /// inside time::hz 



use embassy_stm32::timer::{ Channel, CountingMode::{EdgeAlignedUp} };
use embassy_stm32::dma::{ ReadableRingBuffer, WritableRingBuffer, Transfer, TransferOptions };
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::gpio::OutputType::{ PushPull, OpenDrain };
use embassy_time::Delay;
use embedded_hal::blocking::delay::DelayMs;
use embassy_stm32::time::hz;


pub struct WS2812B <TIM, STREAM, PINS, CLK: Pin,  const STR_CHAN: u8, const TIM_CHAN: u8, const FREQ: u32>
where
    PINS: Pins<TIM, Ch<TIM_CHAN, false>>,
    STREAM: Stream,
    TIM: PwmExt + DmaCcrTimer<TIM_CHAN>,
    PINS::Channels: PwmPin,
    <PINS::Channels as PwmPin>::Duty: From<u16>,
    ChannelX<STR_CHAN>: Channel,
    CCR<TIM, TIM_CHAN>: PeriAddress + DMASet<STREAM, STR_CHAN, MemoryToPeripheral>,
    <CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize: From<u16> + Word + 'static + Copy,

{
    stream: STREAM,
    tim: PwmHz<TIM, Ch<TIM_CHAN, false>, PINS>,
    pins: PINS::Channels,
    clk: Output<'d, CLK>,
    buf: &'static mut [<CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize],
    duty_0: <CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize,
    duty_1: <CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize,
    break_length: usize,
}




/// hal driver
pub struct Ws2812Pwm<TIM, STREAM, PINS, const STR_CHAN: u8, const TIM_CHAN: u8, const FREQ: u32>
where
    PINS: Pins<TIM, Ch<TIM_CHAN, false>>,
    STREAM: Stream,
    TIM: PwmExt + DmaCcrTimer<TIM_CHAN>,
    PINS::Channels: PwmPin,
    <PINS::Channels as PwmPin>::Duty: From<u16>,
    ChannelX<STR_CHAN>: Channel,
    CCR<TIM, TIM_CHAN>: PeriAddress + DMASet<STREAM, STR_CHAN, MemoryToPeripheral>,
    <CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize: From<u16> + Word + 'static + Copy,
{
    stream: STREAM,
    tim: PwmHz<TIM, Ch<TIM_CHAN, false>, PINS>,
    pins: PINS::Channels,
    buf: &'static mut [<CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize],
    duty_0: <CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize,
    duty_1: <CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize,
    break_length: usize,
    //    stop_length: u32,
}

impl<TIM, STREAM, PINS, const STR_CHAN: u8, const TIM_CHAN: u8, const FREQ: u32>
    Ws2812Pwm<TIM, STREAM, PINS, STR_CHAN, TIM_CHAN, FREQ>
where
    PINS: Pins<TIM, Ch<TIM_CHAN, false>>,
    STREAM: Stream,
    TIM: PwmExt + DmaCcrTimer<TIM_CHAN>,
    PINS::Channels: PwmPin,
    <PINS::Channels as PwmPin>::Duty: From<u16>,
    ChannelX<STR_CHAN>: Channel,
    CCR<TIM, TIM_CHAN>: PeriAddress + DMASet<STREAM, STR_CHAN, MemoryToPeripheral>,
    <CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize: From<u16> + Word + 'static + Copy,
{
    pub fn new(
        tim: TIM,
        pins: PINS,
        stream: STREAM,
        buf: &'static mut [<CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize],
        clocks: &Clocks,
    ) -> Self {
        let p_addr = Self::tim_to_ccr(&tim);

        let mut tim = tim.pwm_hz(pins, 800.kHz(), clocks);
        tim.set_polarity(
            stm32f4xx_hal::timer::Channel::C2,
            stm32f4xx_hal::timer::Polarity::ActiveHigh,
        );
        let mut pins = PINS::split();
        pins.set_duty(0.into());
        pins.enable();
        TIM::enable_dma();

        let max_duty = tim.get_max_duty();
        let duty_0 = ((max_duty as u32 * 45 / 125) as u16).into();
        let duty_1 = ((max_duty as u32 * 80 / 125) as u16).into();

        let break_length = 8;

        let stream = Self::configure_stream(stream, p_addr.address());
        Self {
            stream,
            buf,
            tim,
            pins,
            duty_0,
            duty_1,
            break_length,
        }
    }

    pub fn release(
        mut self,
    ) -> (
        TIM,
        STREAM,
        &'static mut [<CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize],
    ) {
        self.pins.disable();
        self.disable_stream();
        TIM::disable_dma();
        return (self.tim.release().release(), self.stream, self.buf);
    }

    fn configure_stream(mut stream: STREAM, p_addr: u32) -> STREAM {
        stream.disable();
        stream.clear_interrupts();
        stream.set_channel::<STR_CHAN>();
        stream.set_direct_mode_error_interrupt_enable(false);
        stream.set_transfer_complete_interrupt_enable(false);
        stream.set_transfer_error_interrupt_enable(false);
        stream.set_fifo_error_interrupt_enable(false);
        stream.set_direction(MemoryToPeripheral);
        stream.set_double_buffer(false);
        stream.set_fifo_enable(true);

        stream.set_memory_increment(true);
        stream.set_memory_burst(stm32f4xx_hal::dma::config::BurstMode::NoBurst);

        stream.set_peripheral_address(p_addr);
        stream.set_peripheral_increment(false);
        stream.set_peripheral_burst(stm32f4xx_hal::dma::config::BurstMode::NoBurst);
        unsafe {
            let memsize = mem::size_of::<<CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize>() as u8 - 1;
            stream.set_memory_size(memsize);
            stream.set_peripheral_size(memsize);
        }
        stream.set_priority(stm32f4xx_hal::dma::config::Priority::Medium);
        stream
    }

    fn disable_stream(&mut self) {
        self.stream.disable();
    }

    fn enable_stream(&mut self, ptr: u32, len: u16) {
        self.stream.set_memory_address(ptr);
        self.stream.set_number_of_transfers(len);
        unsafe {
            self.stream.enable();
        }
    }

    fn tim_to_ccr(tim: &TIM) -> CCR<TIM, TIM_CHAN> {
        assert_eq!(size_of::<TIM>(), size_of::<CCR<TIM, TIM_CHAN>>());
        unsafe { mem::transmute_copy(&tim) }
    }
}

impl<TIM, STREAM, PINS, const STR_CHAN: u8, const TIM_CHAN: u8, const FREQ: u32> SmartLedsWrite
    for Ws2812Pwm<TIM, STREAM, PINS, STR_CHAN, TIM_CHAN, FREQ>
where
    PINS: Pins<TIM, Ch<TIM_CHAN, false>>,
    STREAM: Stream,
    TIM: PwmExt + DmaCcrTimer<TIM_CHAN>,
    PINS::Channels: PwmPin,
    <PINS::Channels as PwmPin>::Duty: From<u16>,
    ChannelX<STR_CHAN>: Channel,
    CCR<TIM, TIM_CHAN>: PeriAddress + DMASet<STREAM, STR_CHAN, MemoryToPeripheral>,
    <CCR<TIM, TIM_CHAN> as PeriAddress>::MemSize: From<u16> + Word + 'static + Copy,
{
    type Error = ();

    type Color = RGB8;

    fn write<T, I>(&mut self, iterator: T) -> Result<(), Self::Error>
    where
        T: Iterator<Item = I>,
        I: Into<Self::Color>,
    {
        self.disable_stream();

        self.stream.clear_interrupts();
        let buf = {
            let bits = iterator
                .flat_map(|c| {
                    let c: RGB8 = c.into();
                    let (r, g, b) = c.into();
                    BitArray::<_, Msb0>::new([g, r, b])
                })
                .map(|b| if b { self.duty_1 } else { self.duty_0 });
            let wait = (0..self.break_length).map(|_| 0.into()); // 0.into() func he;
            let mut i = 0;
            for d in wait.chain(bits) {
                if i == self.buf.len() {
                    break;
                }
                self.buf[i] = d.into();
                i += 1;
            }
            self.buf[i] = 0.into();
            &self.buf[0..i + 1]
        };
        self.enable_stream(buf.as_ptr() as u32, buf.len() as u16);
        Ok(())
    }
}
