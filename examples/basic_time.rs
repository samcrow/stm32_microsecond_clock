#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate panic_rtt_target;
extern crate rtt_target;
extern crate stm32_microsecond_clock;
extern crate stm32f4xx_hal;

use cortex_m_rt::entry;
use stm32f4xx_hal::delay::Delay;
use stm32f4xx_hal::hal::blocking::delay::DelayMs;
use stm32f4xx_hal::interrupt;
use stm32f4xx_hal::rcc::RccExt;

#[entry]
fn main() -> ! {
    rtt_target::rtt_init_print!();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();

    let clocks = dp.RCC.constrain().cfgr.freeze();

    stm32_microsecond_clock::init(dp.TIM6, clocks);
    let mut delay = Delay::new(cp.SYST, clocks);

    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM6_DACUNDER);
    }
    loop {
        delay.delay_ms(1000_u32);
        rtt_target::rprintln!(
            "Current time: {} microseconds",
            stm32_microsecond_clock::microseconds_now().unwrap()
        );
    }
}

// Reminder: As explained in the library documentation, if you use any other interrupts that
// call stm32_microsecond_clock::microseconds_now(), they must have lower priority than this
// timer interrupt.
#[interrupt]
fn TIM6_DACUNDER() {
    stm32_microsecond_clock::handle_timer_overflow();
}
