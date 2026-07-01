/*============================================================================
 * bldc_pwm.h  -- bare-metal driver for the BLDC/PMSM PWM Custom IP (E203 ICB)
 *
 * Memory-mapped 32-bit register interface. Set BLDC_PWM_BASE to the address
 * the IP is mapped to in your E203 SoC, and BLDC_CORE_HZ to the core clock
 * that feeds the IP's `clk`.
 *==========================================================================*/
#ifndef BLDC_PWM_H
#define BLDC_PWM_H

#include <stdint.h>

/* ---- platform configuration (override at build time with -D...) ---- */
#ifndef BLDC_PWM_BASE
#define BLDC_PWM_BASE   0x10040000u   /* ICB slave base address of the IP   */
#endif
#ifndef BLDC_CORE_HZ
#define BLDC_CORE_HZ    16000000u     /* clk frequency feeding the IP (Hz)  */
#endif

/* ---- register offsets ---- */
#define BLDC_CTRL       0x00u
#define BLDC_STATUS     0x04u
#define BLDC_PSC        0x08u
#define BLDC_ARR        0x0Cu
#define BLDC_OCR0       0x10u   /* phase U high-side reference compare */
#define BLDC_OCR1       0x14u
#define BLDC_OCR2       0x18u   /* phase V */
#define BLDC_OCR3       0x1Cu
#define BLDC_OCR4       0x20u   /* phase W */
#define BLDC_OCR5       0x24u
#define BLDC_DTG        0x28u   /* dead-time: [7:0]=U [15:8]=V [23:16]=W */
#define BLDC_CNT        0x2Cu   /* RO */
#define BLDC_SFREQ      0x30u   /* sine phase-accumulator step (speed) */
#define BLDC_SAMP       0x34u   /* sine amplitude / drive strength */
#define BLDC_SCTRL      0x38u
#define BLDC_ID         0x3Cu   /* RO, expect 0xB1DC0001 */

#define BLDC_ID_VALUE   0xB1DC0001u

/* ---- CTRL bit fields ---- */
#define CTRL_EN         (1u << 0)
#define CTRL_MOE        (1u << 1)   /* main output enable (gates all 6 pins) */
#define CTRL_CMS        (1u << 2)   /* 1 = center-aligned, 0 = edge-aligned  */
#define CTRL_AUTO       (1u << 3)   /* hardware sine/SVPWM injection         */
#define CTRL_PRELOAD    (1u << 4)   /* OCR shadow, update at UEV             */
#define CTRL_UIE        (1u << 5)   /* update interrupt enable               */
#define CTRL_BIE        (1u << 6)   /* break interrupt enable                */
#define CTRL_BRK_POL    (1u << 7)   /* break input polarity (0 = act-high)   */
#define CTRL_BRK_SW     (1u << 8)   /* software break (1 = force break)      */
#define CTRL_INDEP      (1u << 9)   /* 1 = 6 independent (NO HW dead-time!)  */
#define CTRL_SVPWM      (1u << 10)  /* 1 = SVPWM table, 0 = pure sine        */

/* ---- STATUS bit fields ---- */
#define STAT_UIF        (1u << 0)   /* update flag,  write 1 to clear */
#define STAT_BIF        (1u << 1)   /* break flag,   write 1 to clear */
#define STAT_BRK_ACT    (1u << 4)   /* break currently asserted (RO)  */

/* ---- low-level register access ---- */
#define BLDC_REG(off)   (*(volatile uint32_t *)(BLDC_PWM_BASE + (off)))

static inline void     bldc_wr(uint32_t off, uint32_t v) { BLDC_REG(off) = v; }
static inline uint32_t bldc_rd(uint32_t off)             { return BLDC_REG(off); }

/* ---- API (see bldc_pwm.c) ---- */

/* Probe the IP; returns 1 if ID matches, 0 otherwise. */
int      bldc_probe(void);

/* Stop & disable everything; outputs forced off (safe state). */
void     bldc_stop(void);

/* Assert / clear the software break. */
void     bldc_break_now(void);
void     bldc_clear_break(void);

/* Return current STATUS register. */
uint32_t bldc_status(void);
/* 1 if a break (HW or SW) is latched in BIF. */
int      bldc_break_pending(void);
/* Clear UIF/BIF latched flags. */
void     bldc_clear_flags(void);

/* Configure the carrier (center-aligned). Picks PSC/ARR for the requested
 * carrier frequency. Returns the achieved carrier in Hz (0 on error). */
uint32_t bldc_set_carrier_hz(uint32_t carrier_hz);

/* Program per-phase dead-time from nanoseconds (errs toward longer = safer).
 * Returns the programmed dead-time count (clk cycles). */
uint32_t bldc_set_deadtime_ns(uint32_t ns);

/* Drive strength as 0..1000 permille of full modulation (ARR/2). */
void     bldc_set_amplitude_permille(uint32_t permille);

/* Electrical frequency in milli-Hz (e.g. 2000 = 2.0 Hz). Returns SFREQ set. */
uint32_t bldc_set_elec_freq_mhz(uint32_t freq_mhz);

/* Start hardware-injected drive. svpwm=1 selects SVPWM table, 0 pure sine.
 * Uses center-aligned + preload + complementary outputs (safe mode). */
void     bldc_start(int svpwm);

#endif /* BLDC_PWM_H */
