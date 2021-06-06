//!
//! A clock for STM32 microcontrollers with microsecond precision and 48-bit capacity
//!
//! # Supported microcontrollers
//!
//! * STM32F412 using TIM6
//!
//! # Requirements for correct behavior
//!
//! This library uses a timer and receives interrupts when the timer overflows. The application
//! must define a handler for the timer interrupt that calls `handle_timer_overflow()`, and does
//! not call `microseconds_now()` or any of its wrapper functions until after
//! `handle_timer_overflow()` is called.
//!
//! The timer interrupt handler must have higher priority than any other interrupt or task that
//! calls `microseconds_now()` or any of its wrapper functions.
//!
//! The timer interrupt must be allowed to execute at least every 65.536 microseconds. Therefore,
//! interrupts may not be disabled for 65.536 milliseconds or longer, and no interrupt or task
//! that has a higher priority than the timer interrupt may run continuously for 65.536 milliseconds
//! or longer.
//!
//! Violating these constraints will not cause undefined behavior, but it may cause
//! `microseconds_now()` to return an incorrect time.
//!
//! # Clock requirements
//!
//! The clock input to the timer must have a frequency between 1 and 65536 megahertz. The init()
//! function will panic if this is not satisfied.
//!
//! For the clock to be accurate, the clock input to the timer should have a frequency that is
//! an integer number of megahertz.
//!

#![no_std]

/// The timer peripheral that the clock uses
#[cfg(feature = "stm32f412")]
pub type Timer = stm32f4xx_hal::pac::TIM6;
#[cfg(feature = "stm32f413")]
pub type Timer = stm32f4xx_hal::pac::TIM6;
#[cfg(not(any(feature = "stm32f412", feature = "stm32f413")))]
compile_error!("stm32_microsecond_clock requires a feature to select the target device");
// Implementation-specific clock information type
#[cfg(any(feature = "stm32f412", feature = "stm32f413"))]
use stm32f4xx_hal::rcc::Clocks;

use core::convert::TryInto;
use embedded_time::clock::Error;
use embedded_time::duration::Fraction;
use embedded_time::{Clock, Instant};

/// The number of times the timer has overflowed
static mut OVERFLOW_COUNT: u32 = 0;
/// If the initialization function has finished
static mut INITIALIZED: bool = false;

pub struct MicrosecondClock;

// embedded-time compatibility
impl Clock for MicrosecondClock {
    type T = u64;
    const SCALING_FACTOR: Fraction = Fraction::new(1, 1_000_000);

    fn try_now(&self) -> Result<Instant<Self>, Error> {
        microseconds_now()
            .map(Instant::new)
            .map_err(|_| Error::NotRunning)
    }
}

/// Initializes the clock using a timer
///
/// After calling this function, the timer interrupt must be unmasked in the nested vector interrupt
/// controller.
pub fn init(timer: Timer, clocks: Clocks) {
    let frequency_in = enable_timer_clock(clocks);

    // Configure prescaler so 1 tick = 1 microsecond (1 megahertz)
    let prescaler = frequency_in / 1_000_000;
    assert!(prescaler > 0, "Clock input to timer 6 is too slow");
    let prescaler_bits: u16 = (prescaler - 1)
        .try_into()
        .expect("Clock input to timer 6 is too fast");
    timer.psc.write(|w| w.psc().bits(prescaler_bits));

    // Reload upon reaching 65536
    // With this setup, the timer will overflow every 65.536 milliseconds.
    #[allow(unused_unsafe)] // ARR_W.bits() is unsafe on some microcontroller models.
    timer.arr.write(|w| unsafe { w.arr().bits(u16::MAX) });

    // Trigger update event to load the registers
    timer.cr1.modify(|_, w| w.urs().counter_only());
    timer.egr.write(|w| w.ug().update());
    timer.cr1.modify(|_, w| w.urs().any_event());

    // Enable update (overflow) interrupt
    timer.dier.modify(|_, w| w.uie().enabled());

    // Start counting
    timer.cr1.modify(|_, w| w.cen().enabled());

    // Indicate to the other code that the clock is ready
    unsafe {
        OVERFLOW_COUNT = 0;
        INITIALIZED = true;
    }
}

/// Returns the number of microseconds since the clock was started
///
/// This value has a capacity of 48 bits. It will overflow after approximately 8 years.
///
/// If the clock has not been started, this function returns an error.
pub fn microseconds_now() -> Result<u64, NotRunning> {
    // Safety explanation:
    // This function can be called from any context, and it may be preempted by other interrupts.
    // It reads INITIALIZED, OVERFLOW_COUNT and the cnt register of the timer.
    //
    // Part 1: INITIALIZED
    // The only place INITIALIZED is written is in the init() function, in an interrupt-free
    // section.
    //
    // Part 2: OVERFLOW_COUNT
    // OVERFLOW_COUNT is read here and read/modified/written in handle_timer_overflow(). This is
    // a possible data race between reading the value here and writing the incremented value in
    // handle_timer_overflow(). However, because of the interrupt priority requirement, this
    // function cannot execute between the timer overflowing and handle_timer_overflow() returning.
    //
    // Part 3: Timer.cnt
    // If INITIALIZED is true, init() was called and given a Timer, so no other code can access
    // the timer. This function accesses only the cnt register, which no other code in this library
    // accesses.

    if unsafe { INITIALIZED } {
        let timer = unsafe { &*Timer::ptr() };
        let overflow_count = unsafe { OVERFLOW_COUNT };
        let microseconds_since_overflow = timer.cnt.read().cnt().bits();
        let time = (u64::from(overflow_count) << 16) | u64::from(microseconds_since_overflow);
        Ok(time)
    } else {
        Err(NotRunning(()))
    }
}

/// An error returned when the clock has not been started
#[derive(Debug)]
pub struct NotRunning(());

/// Handles an interrupt from the timer
pub fn handle_timer_overflow() {
    // Safety explanation:
    // This function should only be called from the timer interrupt, which has higher priority
    // than any code that calls microseconds_now(). It reads INITIALIZED, reads/modifies/writes
    // Timer.UIF, and reads/modifies/writes OVERFLOW_COUNT.
    //
    // No other code accesses Timer.UIF. No other code writes INITIALIZED after init() returns.
    // No other code writes OVERFLOW_COUNT.
    //
    // Because of the interrupt priority rule, no other code can read OVERFLOW_COUNT in between
    // when the timer overflows and when this function returns.
    unsafe {
        if INITIALIZED {
            let timer = &*Timer::ptr();
            if timer.sr.read().uif().bit_is_set() {
                // Clear interrupt flag
                timer.sr.write(|w| w.uif().clear_bit());
                OVERFLOW_COUNT = OVERFLOW_COUNT.wrapping_add(1);
            }
        }
    }
}

/// Enables the clock for the timer and returns its frequency in Hertz
#[cfg(any(feature = "stm32f412", feature = "stm32f413"))]
fn enable_timer_clock(clocks: Clocks) -> u32 {
    unsafe {
        use stm32f4xx_hal::bb;
        use stm32f4xx_hal::pac::RCC;
        let rcc = &*RCC::ptr();
        bb::set(&rcc.apb1enr, 4);
        cortex_m::asm::dsb();
        bb::set(&rcc.apb1rstr, 4);
        bb::clear(&rcc.apb1rstr, 4);
    }
    let pclk_mul = if clocks.ppre1() == 1 { 1 } else { 2 };
    let frequency_in = clocks.pclk1().0 * pclk_mul;
    frequency_in
}
