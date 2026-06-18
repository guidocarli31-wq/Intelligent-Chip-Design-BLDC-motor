#!/usr/bin/env python3
"""
Generate look-up tables for the BLDC PWM sine/SVPWM injector.

Produces two hex files of signed Q15 samples (16-bit two's complement, one
hex word per line) with N entries spanning one electrical period [0, 2*pi):

    sine_lut.hex   : pure sine,  amplitude +/-1.0
    svpwm_lut.hex  : SVPWM (min/max common-mode injection), normalized so its
                     peak reaches +/-1.0 -> ~15% more linear range than sine.

The injector reads ONE table at three indices 120 deg apart, so a single
phase's waveform reproduces all three phases by symmetry.
"""
import math

N      = 256          # table length (must equal 2^LAW in the RTL; LAW=8)
Q      = 32767        # Q15 full-scale
TWO_PI = 2.0 * math.pi


def to_q15_hex(x):
    """Clamp x in [-1,1] to signed Q15 and return 4-digit hex (16-bit)."""
    v = int(round(max(-1.0, min(1.0, x)) * Q))
    if v < 0:
        v += 1 << 16          # two's complement
    return "{:04X}".format(v & 0xFFFF)


def main():
    sine = []
    svpwm = []
    for i in range(N):
        th = TWO_PI * i / N
        a = math.sin(th)
        b = math.sin(th - TWO_PI / 3.0)
        c = math.sin(th + TWO_PI / 3.0)
        common = 0.5 * (max(a, b, c) + min(a, b, c))
        sv = (a - common) * (2.0 / math.sqrt(3.0))   # normalize peak to 1.0
        sine.append(a)
        svpwm.append(sv)

    with open("sine_lut.hex", "w") as f:
        f.write("\n".join(to_q15_hex(x) for x in sine) + "\n")
    with open("svpwm_lut.hex", "w") as f:
        f.write("\n".join(to_q15_hex(x) for x in svpwm) + "\n")

    print("wrote sine_lut.hex and svpwm_lut.hex ({} entries each)".format(N))


if __name__ == "__main__":
    main()
