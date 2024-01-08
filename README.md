Current issues that need fixing:
- if i want to turn the crate into asycnh  need to add an open drain? so also an additional resistor

Reminders:
dma:
- arr - auto reload register - these are the TIMs which have channels; responsible for interrupts - https://ziblog.ru/2011/01/15/stm32-chast-8-ndash-taymeryi-obshhego-naznacheniya-preryivaniya.html
- pre-scalar - cuts the frequency of timer ticks - TIM1 & TIM8, while 2-5 are general-purpose timers; others are basic
- stm32 metapac - crate for working with dma stream mappings, interrupts, etc - https://crates.io/crates/stm32-metapac
- embassy-time is currently used and is tied to the tick rate of the current driver to mathc the hardware - this crate allows for asynch delays


pwm:
- https://docs.embassy.dev/embassy-stm32/git/stm32f030c6/timer/index.html