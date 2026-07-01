/*============================================================================
 * bldc_pwm.c  -- driver implementation for the BLDC/PMSM PWM Custom IP
 *==========================================================================*/
#include "bldc_pwm.h"

/* cached timing so amplitude/frequency helpers can do their math */
static uint32_t g_arr  = 1023;
static uint32_t g_fuev = 1;       /* update-event frequency (= carrier), Hz */

int bldc_probe(void)
{
    return (bldc_rd(BLDC_ID) == BLDC_ID_VALUE) ? 1 : 0;
}

void bldc_stop(void)
{
    /* EN=0 and MOE=0 -> counter halts and all six outputs forced off. */
    bldc_wr(BLDC_CTRL, 0u);
}

void bldc_break_now(void)
{
    bldc_wr(BLDC_CTRL, bldc_rd(BLDC_CTRL) | CTRL_BRK_SW);
}

void bldc_clear_break(void)
{
    bldc_wr(BLDC_CTRL, bldc_rd(BLDC_CTRL) & ~CTRL_BRK_SW);
    bldc_wr(BLDC_STATUS, STAT_BIF);          /* W1C the latched flag */
}

uint32_t bldc_status(void)        { return bldc_rd(BLDC_STATUS); }
int      bldc_break_pending(void) { return (bldc_rd(BLDC_STATUS) & STAT_BIF) ? 1 : 0; }
void     bldc_clear_flags(void)   { bldc_wr(BLDC_STATUS, STAT_UIF | STAT_BIF); }

uint32_t bldc_set_carrier_hz(uint32_t carrier_hz)
{
    if (carrier_hz == 0u) return 0u;

    /* center-aligned: carrier = CORE / (2 * ARR * (PSC+1)) */
    uint32_t total = BLDC_CORE_HZ / (2u * carrier_hz);   /* = ARR*(PSC+1) */
    if (total == 0u) total = 1u;

    uint32_t psc = 0u;
    uint32_t arr = total;
    while (arr > 0xFFFFu) {           /* keep ARR within 16 bits */
        psc++;
        arr = total / (psc + 1u);
    }
    if (arr < 2u) arr = 2u;

    bldc_wr(BLDC_PSC, psc);
    bldc_wr(BLDC_ARR, arr);

    g_arr  = arr;
    g_fuev = BLDC_CORE_HZ / (2u * arr * (psc + 1u));
    if (g_fuev == 0u) g_fuev = 1u;
    return g_fuev;
}

uint32_t bldc_set_deadtime_ns(uint32_t ns)
{
    /* cycles = ns * CORE_HZ / 1e9, rounded up; clamp to [1,255] per phase. */
    uint64_t cyc = ((uint64_t)ns * (uint64_t)BLDC_CORE_HZ + 999999999ull)
                   / 1000000000ull;
    if (cyc < 1u)   cyc = 1u;
    if (cyc > 255u) cyc = 255u;

    uint32_t d = (uint32_t)cyc;
    bldc_wr(BLDC_DTG, d | (d << 8) | (d << 16));   /* U | V | W */
    return d;
}

void bldc_set_amplitude_permille(uint32_t permille)
{
    if (permille > 1000u) permille = 1000u;
    uint32_t samp = (g_arr / 2u) * permille / 1000u;   /* 0..ARR/2 */
    bldc_wr(BLDC_SAMP, samp);
}

uint32_t bldc_set_elec_freq_mhz(uint32_t freq_mhz)
{
    /* SFREQ = f_e / f_uev * 2^32 ; f_e[Hz] = freq_mhz/1000 */
    uint64_t sfreq = ((uint64_t)freq_mhz << 32) / (1000ull * (uint64_t)g_fuev);
    bldc_wr(BLDC_SFREQ, (uint32_t)sfreq);
    return (uint32_t)sfreq;
}

void bldc_start(int svpwm)
{
    uint32_t ctrl = CTRL_EN | CTRL_MOE | CTRL_CMS | CTRL_AUTO | CTRL_PRELOAD;
    if (svpwm) ctrl |= CTRL_SVPWM;
    /* INDEP intentionally left clear: hardware complementary + dead-time. */
    bldc_wr(BLDC_CTRL, ctrl);
}
