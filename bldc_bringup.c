/*============================================================================
 * bldc_bringup.c  -- GUARDED first-spin bring-up for the BLDC PWM IP.
 *
 * Walks Phases B (powered, current-limited, zero drive -> align) and
 * C (slow V/f ramp), checking for a latched BREAK at every step and stopping
 * immediately if one occurs.
 *
 * >>> READ THE CHECKLIST (docs/bringup_checklist.md) AND COMPLETE PHASE A
 *     (scope dead-time, verify output polarity, verify break) BEFORE running
 *     this with the power stage energized. <<<
 *
 * Keep the bench supply CURRENT-LIMITED and watch the meter throughout.
 *==========================================================================*/
#include "bldc_pwm.h"

/* ---- tunables: start conservative, adjust for your motor/driver ---- */
#define CARRIER_HZ        16000u   /* PWM carrier frequency               */
#define DEADTIME_NS        1000u   /* MUST be >= your MOSFET/driver needs  */
#define USE_SVPWM             1    /* 1 = SVPWM table, 0 = pure sine       */

#define ALIGN_PERMILLE       80u   /* drive strength while parking rotor   */
#define ALIGN_MS            500u

#define F_START_MHZ        1000u   /* 1.000 Hz electrical                  */
#define F_TARGET_MHZ       5000u   /* 5.000 Hz electrical (~speed target)  */
#define AMP_MIN_PERMILLE     80u   /* V/f: amplitude at F_START            */
#define AMP_MAX_PERMILLE    250u   /* V/f: amplitude at F_TARGET            */
#define RAMP_STEPS           40u
#define STEP_MS             100u   /* -> ramp takes RAMP_STEPS*STEP_MS ms  */
#define HOLD_MS            3000u

/* ---- logging hook (define BLDC_HAS_PRINTF and provide printf) ---- */
#ifdef BLDC_HAS_PRINTF
#include <stdio.h>
#define LOG(...)  printf(__VA_ARGS__)
#else
#define LOG(...)  do {} while (0)
#endif

/* ---- delay hook: replace with your timer-based delay for accuracy ---- */
__attribute__((weak)) void bldc_delay_ms(uint32_t ms)
{
    /* crude busy-wait fallback (~ approximate); override for real timing */
    volatile uint64_t n = (uint64_t)ms * (BLDC_CORE_HZ / 1000u) / 8u;
    while (n--) { __asm__ volatile (""); }
}

/* Stop and return on a latched break. */
static int abort_if_break(const char *where)
{
    (void)where;
    if (bldc_break_pending()) {
        bldc_stop();
        LOG("ABORT: BREAK at %s\n", where);
        return 1;
    }
    return 0;
}

/* Returns 0 on a clean run, negative on abort. */
int bldc_bringup_run(void)
{
    uint32_t i, f, a;

    /* ---- sanity ---- */
    if (!bldc_probe()) { LOG("ERROR: IP id mismatch\n"); return -1; }
    bldc_stop();
    bldc_clear_flags();

    uint32_t car = bldc_set_carrier_hz(CARRIER_HZ);
    uint32_t dtg = bldc_set_deadtime_ns(DEADTIME_NS);
    (void)car; (void)dtg;
    LOG("carrier=%lu Hz, deadtime=%lu cyc\n",
        (unsigned long)car, (unsigned long)dtg);

    /* ---- Phase B7: enable with ZERO drive (50%% duty, no net voltage) ---- */
    bldc_set_amplitude_permille(0u);
    bldc_set_elec_freq_mhz(0u);
    bldc_start(USE_SVPWM);          /* complementary + dead-time (safe mode) */
    bldc_delay_ms(200u);
    if (abort_if_break("zero-drive")) return -2;
    /* >>> operator: confirm bus current is ~0 here before continuing <<< */

    /* ---- Phase B8: align rotor on a fixed vector ---- */
    bldc_set_elec_freq_mhz(0u);
    bldc_set_amplitude_permille(ALIGN_PERMILLE);
    bldc_delay_ms(ALIGN_MS);
    if (abort_if_break("align")) return -3;

    /* ---- Phase C9: slow V/f ramp (open-loop) ---- */
    for (i = 0u; i <= RAMP_STEPS; i++) {
        f = F_START_MHZ + (F_TARGET_MHZ - F_START_MHZ) * i / RAMP_STEPS;
        a = AMP_MIN_PERMILLE + (AMP_MAX_PERMILLE - AMP_MIN_PERMILLE) * i / RAMP_STEPS;
        bldc_set_amplitude_permille(a);   /* set amplitude before freq */
        bldc_set_elec_freq_mhz(f);
        bldc_delay_ms(STEP_MS);
        if (abort_if_break("ramp")) return -4;
        /* >>> operator: watch current; if it climbs, stop and lower AMP <<< */
    }
    LOG("at target: f=%lu mHz, amp=%lu permille\n",
        (unsigned long)F_TARGET_MHZ, (unsigned long)AMP_MAX_PERMILLE);

    /* ---- hold ---- */
    bldc_delay_ms(HOLD_MS);
    if (abort_if_break("hold")) return -5;

    /* ---- graceful ramp-down then stop ---- */
    for (i = RAMP_STEPS; i > 0u; i--) {
        f = F_START_MHZ + (F_TARGET_MHZ - F_START_MHZ) * i / RAMP_STEPS;
        a = AMP_MIN_PERMILLE + (AMP_MAX_PERMILLE - AMP_MIN_PERMILLE) * i / RAMP_STEPS;
        bldc_set_amplitude_permille(a);
        bldc_set_elec_freq_mhz(f);
        bldc_delay_ms(STEP_MS / 2u);
    }
    bldc_stop();
    LOG("done, stopped\n");
    return 0;
}

#ifdef BLDC_BRINGUP_MAIN
int main(void)
{
    int rc = bldc_bringup_run();
    if (rc != 0) {
        bldc_stop();          /* make sure the bridge is off on any abort */
    }
    for (;;) { }
    return rc;
}
#endif
