/*============================================================================
 * bldc_phaseA.c  -- bench helpers for Phase A (NO MOTOR, NO HIGH VOLTAGE).
 *
 * Use these to verify dead-time, output polarity, and BREAK on a scope before
 * the power stage is ever energized.
 *==========================================================================*/
#include "bldc_pwm.h"

/* Force all six outputs to logic-low (MOE=0). Use to check that your gate
 * driver reads logic-low as transistor OFF (polarity check). */
void bldc_outputs_off(void)
{
    bldc_wr(BLDC_CTRL, 0u);
}

/* Drive a steady ~50%% complementary PWM on U/V/W (edge-aligned, manual mode,
 * hardware dead-time) so you can scope the high/low gate pair and MEASURE the
 * dead-time window (time where both gates are low at each switching edge).
 *
 *   carrier_hz  : PWM frequency to display
 *   deadtime_ns : dead-time to program (the value you intend to ship)
 */
void bldc_phaseA_scope(uint32_t carrier_hz, uint32_t deadtime_ns)
{
    bldc_stop();
    bldc_clear_flags();

    (void)bldc_set_carrier_hz(carrier_hz);
    (void)bldc_set_deadtime_ns(deadtime_ns);

    uint32_t arr  = bldc_rd(BLDC_ARR);
    uint32_t half = arr >> 1;                 /* ~50% duty */
    bldc_wr(BLDC_OCR0, half);                 /* phase U */
    bldc_wr(BLDC_OCR2, half);                 /* phase V */
    bldc_wr(BLDC_OCR4, half);                 /* phase W */

    /* EN | MOE | PRELOAD : edge-aligned, AUTO off, INDEP off (complementary) */
    bldc_wr(BLDC_CTRL, CTRL_EN | CTRL_MOE | CTRL_PRELOAD);
    /* Scope now: U_high vs U_low must never overlap; measure the both-low gap. */
}

/* Assert the software BREAK and report whether it latched. While
 * bldc_phaseA_scope() is running, call this and confirm on the scope that all
 * six outputs go OFF. Returns 1 if the break flag latched. */
int bldc_phaseA_break_test(void)
{
    bldc_break_now();                 /* same effect as pulling the brk_in pin */
    return bldc_break_pending();
}

/* Recover from a break test back into the scope state. */
void bldc_phaseA_break_clear(void)
{
    bldc_clear_break();
}
