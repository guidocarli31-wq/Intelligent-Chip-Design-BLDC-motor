/*============================================================================
 * bldc_delay.c  -- accurate bldc_delay_ms() using the RISC-V cycle counter.
 *
 * Overrides the weak busy-loop fallback in bldc_bringup.c. Depends only on
 * BLDC_CORE_HZ (the clock feeding the IP / core), which you already set in
 * bldc_pwm.h. Reads the 64-bit mcycle counter so it never wraps in practice.
 *
 * If you prefer the machine timer (CLINT mtime, low-power), see the note at
 * the bottom -- but mtime runs at the RTC tick rate, not the core clock, so
 * you would scale by that frequency instead of BLDC_CORE_HZ.
 *==========================================================================*/
#include "bldc_pwm.h"

#if defined(__riscv)

static inline uint32_t rd_mcycle(void)
{ uint32_t v; __asm__ volatile ("csrr %0, mcycle"  : "=r"(v)); return v; }
static inline uint32_t rd_mcycleh(void)
{ uint32_t v; __asm__ volatile ("csrr %0, mcycleh" : "=r"(v)); return v; }

/* Glitch-free 64-bit read on RV32 (re-read high half on rollover). */
static uint64_t rd_mcycle64(void)
{
    uint32_t hi, lo, hi2;
    do { hi = rd_mcycleh(); lo = rd_mcycle(); hi2 = rd_mcycleh(); }
    while (hi != hi2);
    return ((uint64_t)hi << 32) | lo;
}

void bldc_delay_ms(uint32_t ms)
{
    uint64_t target = rd_mcycle64()
                    + (uint64_t)ms * (uint64_t)(BLDC_CORE_HZ / 1000u);
    while (rd_mcycle64() < target) { /* spin */ }
}

#else  /* host build: lets the firmware compile for syntax checks only */

void bldc_delay_ms(uint32_t ms)
{
    static volatile uint64_t fake;
    uint64_t target = fake + (uint64_t)ms * (BLDC_CORE_HZ / 1000u);
    while (fake < target) { fake++; }
}

#endif

/*
 * Alternative using CLINT mtime (standard RISC-V machine timer):
 *
 *   #define CLINT_BASE   0x02000000u
 *   #define MTIME_LO     (CLINT_BASE + 0xBFF8u)
 *   #define MTIME_HI     (CLINT_BASE + 0xBFFCu)
 *   // delay = ms * MTIME_HZ / 1000, where MTIME_HZ is your RTC tick rate.
 *
 * Verify CLINT_BASE and the mtime tick frequency against YOUR SoC memory map.
 */
