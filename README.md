# BLDC / PMSM PWM Custom IP for Hummingbird E203 (ICB)

A 16-bit advanced timer / PWM peripheral that generates 3-phase complementary
PWM with hardware dead-time, emergency BREAK, and optional hardware sine/SVPWM
injection. It attaches to the E203 peripheral **ICB** bus as a 32-bit register
slave.

This package covers **all** the project requirements (basic + extended):

| Req | Feature | Where |
|-----|---------|-------|
| 1 | 3-phase complementary PWM (6 signals, 3 pairs) | `bldc_pwm_timer.v` + `bldc_deadtime_gen.v` |
| 2 | Per-phase dead-zone, software-configurable | `DTG` register -> `bldc_deadtime_gen.v` |
| 3 | Periodic OC update (shadow/preload + UEV IRQ) | `preload` + `uev`/`UIF` |
| 4 | Emergency BREAK (ASAP shutdown) | `brk_in` / `BRK_SW`, combinational output mask |
| 5 | Automatic OC injection from a sine table (HW) | `bldc_sine_injector.v` |
| 6 | Driving-strength control | `SAMP` (amplitude / modulation depth) |
| 7 | Speed control | `SFREQ` (phase-accumulator step) and/or `PSC`/`ARR` |

## Files

```
rtl/
  bldc_pwm_icb_top.v     ICB slave + register file + OCR mux + IRQ (TOP)
  bldc_pwm_timer.v       16-bit counter, compare, complementary+DT, BREAK
  bldc_deadtime_gen.v    one half-bridge: complement + programmable dead-time
  bldc_sine_injector.v   phase-accum NCO + sine/SVPWM LUT + amplitude scale
sim/
  tb_bldc_pwm.v          smoke testbench (manual PWM, BREAK, auto-inject)
  sine_lut.hex           256-entry signed Q15 sine LUT   (loaded by injector)
  svpwm_lut.hex          256-entry signed Q15 SVPWM LUT
  wave_*.png             verification waveforms (from golden_model.py)
scripts/
  gen_sine_lut.py        regenerate the .hex LUTs
  golden_model.py        cycle-accurate Python twin + numeric checks + plots
```

## Register map (32-bit, word-aligned; base = peripheral base address)

| Offset | Name   | Access | Description |
|--------|--------|--------|-------------|
| 0x00 | CTRL    | RW  | control bits (below) |
| 0x04 | STATUS  | RW  | status flags (W1C on UIF/BIF) |
| 0x08 | PSC     | RW  | prescaler-1: timer tick = clk / (PSC+1) `[15:0]` |
| 0x0C | ARR     | RW  | auto-reload / period top `[15:0]` |
| 0x10 | OCR0    | RW  | compare, phase **U** high-side ref `[15:0]` |
| 0x14 | OCR1    | RW  | compare ch1 (independent mode only) |
| 0x18 | OCR2    | RW  | compare, phase **V** high-side ref `[15:0]` |
| 0x1C | OCR3    | RW  | compare ch3 (independent mode only) |
| 0x20 | OCR4    | RW  | compare, phase **W** high-side ref `[15:0]` |
| 0x24 | OCR5    | RW  | compare ch5 (independent mode only) |
| 0x28 | DTG     | RW  | dead-time `[7:0]=U  [15:8]=V  [23:16]=W` (clk cycles) |
| 0x2C | CNT     | RO  | live counter value `[15:0]` |
| 0x30 | SFREQ   | RW  | sine phase-accumulator step `[31:0]` (speed) |
| 0x34 | SAMP    | RW  | sine amplitude / drive strength `[15:0]` (0..ARR/2) |
| 0x38 | SCTRL   | RW  | reserved for expansion |
| 0x3C | ID      | RO  | `0xB1DC0001` (probe / version) |

### CTRL bits

| Bit | Name | Meaning |
|-----|------|---------|
| 0 | EN      | counter enable |
| 1 | MOE     | main output enable (gates all 6 outputs) |
| 2 | CMS     | 1 = center-aligned (triangular), 0 = edge-aligned (saw) |
| 3 | AUTO    | enable hardware sine/SVPWM injection -> drives OCR0/2/4 |
| 4 | PRELOAD | OCR shadow: new values take effect only at the Update Event |
| 5 | UIE     | update-event interrupt enable |
| 6 | BIE     | break interrupt enable |
| 7 | BRK_POL | break input polarity (0 = active high) |
| 8 | BRK_SW  | software break (1 = force break now) |
| 9 | INDEP   | 1 = 6 independent outputs (NO hardware complement/dead-time) |
| 10| SVPWM   | injector table select: 0 = pure sine, 1 = SVPWM (3rd-harm) |

### STATUS bits

| Bit | Name | Notes |
|-----|------|-------|
| 0 | UIF     | update-event flag, write 1 to clear |
| 1 | BIF     | break flag, write 1 to clear |
| 4 | BRK_ACT | 1 = break currently asserted (RO) |

## Output pin map (`pwm_out[5:0]`)

```
[0]=U_high [1]=U_low   [2]=V_high [3]=V_low   [4]=W_high [5]=W_low
```

## ICB interface

Standard E203 command/response bus with valid-ready handshakes, 32-bit data:

```
icb_cmd_valid / icb_cmd_ready / icb_cmd_addr / icb_cmd_read /
icb_cmd_wdata / icb_cmd_wmask
icb_rsp_valid / icb_rsp_ready / icb_rsp_rdata / icb_rsp_err
```

The slave keeps a single outstanding transaction; registers respond one cycle
after the command handshake. This matches the E203 peripheral ICB fabric.

### Mounting on E203

1. Instantiate `bldc_pwm_icb_top` inside the E203 peripheral subsystem
   (`e203_subsys_perips` / the ICB cross-bar) and give it a slave region, e.g.
   a 4 KB window at `0x1004_0000`. Wire its ICB port to a free ICB slave branch
   (the E203 SoC already splits the peripheral ICB to GPIO/UART/etc.).
2. Connect `irq` to a free **PLIC** interrupt source line.
3. Route `pwm_out[5:0]` to package / FPGA pins that feed the external 3-phase
   gate-driver board (the `bldc_pwr_drv` power stage).
4. Drive `brk_in` from the driver's fault / over-current comparator output.
5. `clk` / `rst_n` come from the E203 always-on / core clock and reset.

## Simulate (Icarus Verilog or Verilator)

Run from `sim/` so the `.hex` LUTs are found by `$readmemh`:

```
cd sim
iverilog -g2012 -o sim.out ../rtl/*.v tb_bldc_pwm.v
vvp sim.out
gtkwave tb_bldc_pwm.vcd      # optional waveform view
```

Regenerate the LUTs if you change table size / shape:

```
cd sim && python3 ../scripts/gen_sine_lut.py
```

## Verification status

A cycle-accurate Python twin (`scripts/golden_model.py`) mirrors the RTL and
checks the algorithm. Current results (see `sim/wave_*.png`):

```
complementary + dead-time : PASS   (0 shoot-through cycles; dead-time = DTG+1 cyc)
break                     : PASS   (all 6 outputs forced off within 2 clk)
sine 3-phase              : PASS   (phases exactly 120/240 deg apart)
svpwm                     : PASS   (per-phase 3rd-harmonic injected, cancels line-to-line)
```

Frequency relationship (verified numerically):

```
f_uev = f_clk / ( (PSC+1) * period )      period = ARR+1 (edge) or 2*ARR (center)
f_elec = f_uev * SFREQ / 2^32             RPM = 60 * f_elec / pole_pairs
```

Example: `f_clk=16 MHz, ARR=800, PSC=0, center-aligned -> f_uev=10 kHz`.
`SFREQ` for 1/5/10 Hz electrical = 429497 / 2147484 / 4294967, giving
60 / 300 / 600 RPM on a 1-pole-pair motor (matches the spec's 1-10 Hz, 60-600 RPM).

> Note: the included testbench is ready to run under iverilog/Verilator; the
> Python twin is the substitute used for algorithm verification when no HDL
> simulator is installed.

## SAFETY — read before connecting a real motor

* **Verify dead-time first (requirement 2).** Confirm in simulation that the
  high/low gates of every phase are never high at the same time, and that the
  blanking window matches your gate driver's needs, BEFORE applying the 12 V
  power stage. Zero/insufficient dead-time shoot-through destroys the MOSFETs.
* Keep `INDEP` mode (CTRL[9]) **off** for a real power stage — it removes the
  hardware complement and dead-time. Use it only for bench signal testing.
* On BREAK (`brk_in` or `BRK_SW`), all six outputs are forced low (both
  transistors off, motor freewheels) combinationally for fastest shutdown.
* Bring up with small `SAMP` (low drive strength) and a current limit.

## Typical bring-up sequence (firmware, conceptual)

```
write PSC, ARR          ; set carrier frequency (1-10 kHz update rate)
write DTG               ; per-phase dead-time (verify on a scope!)
write SAMP (small)      ; low drive strength
write SFREQ             ; electrical frequency (speed)
write CTRL = EN|MOE|CMS|AUTO|PRELOAD|SVPWM   ; start with HW SVPWM injection
; increase SAMP for torque, change SFREQ for speed
; on fault -> hardware brk_in (or write CTRL.BRK_SW) shuts down immediately
```
