#!/usr/bin/env python3
"""
Cycle-accurate Python twin of the BLDC PWM RTL (bldc_pwm_timer +
bldc_deadtime_gen + bldc_sine_injector). Mirrors the exact register logic so
we can verify the design without an HDL simulator, run numeric checks, and
emit waveform plots for the report.
"""
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------- LUTs (same as gen_sine_lut.py) ----------
N = 256
TWO_PI = 2 * math.pi
def build_luts():
    sine, svpwm = [], []
    for i in range(N):
        th = TWO_PI * i / N
        a = math.sin(th); b = math.sin(th-TWO_PI/3); c = math.sin(th+TWO_PI/3)
        common = 0.5*(max(a,b,c)+min(a,b,c))
        sv = (a-common)*(2/math.sqrt(3))
        sine.append(int(round(max(-1,min(1,a))*32767)))
        svpwm.append(int(round(max(-1,min(1,sv))*32767)))
    return sine, svpwm
SINE, SVPWM = build_luts()

def sgn16(x):  # interpret as signed 16-bit
    return x-65536 if x >= 32768 else x

# ---------- dead-time generator (mirror of bldc_deadtime_gen.v) ----------
class DTGen:
    def __init__(self):
        self.ref_d=0; self.dcnt=0; self.in_dt=0; self.oh=0; self.ol=0
    def step(self, ena, ref_in, dt):
        if not ena:
            n=dict(ref_d=ref_in,dcnt=0,in_dt=0,oh=0,ol=0)
        else:
            edge = ref_in ^ self.ref_d
            if edge:
                if dt!=0: n=dict(ref_d=ref_in,in_dt=1,dcnt=dt,oh=0,ol=0)
                else:     n=dict(ref_d=ref_in,in_dt=0,dcnt=0,oh=0,ol=0)
            elif self.in_dt:
                if self.dcnt>1: n=dict(ref_d=ref_in,in_dt=1,dcnt=self.dcnt-1,oh=0,ol=0)
                else:           n=dict(ref_d=ref_in,in_dt=0,dcnt=0,oh=0,ol=0)
            else:
                n=dict(ref_d=ref_in,in_dt=0,dcnt=0,oh=ref_in,ol=0 if ref_in else 1)
        return n
    def commit(self,n): self.__dict__.update(n)

# ---------- injector (mirror of bldc_sine_injector.v) ----------
class Injector:
    def __init__(self): self.phase=0; self.u=0; self.v=0; self.w=0
    def calc(self, idx_phase, table, amp, arr):
        idx=(idx_phase>>24)&0xFF
        s=sgn16(table[idx]&0xFFFF) if isinstance(table[idx],int) and table[idx]>=0 else table[idx]
        s=table[idx]
        prod=amp*s
        scaled=prod>>15 if prod>=0 else -((-prod)>>15)  # arithmetic >>15
        calc=(arr>>1)+scaled
        return max(0,min(arr,calc))
    def step(self, en, load, svpwm_sel, step, amp, arr):
        table=SVPWM if svpwm_sel else SINE
        if not en:
            self.phase=0; self.u=self.v=self.w=arr>>1; return
        if load:
            self.u=self.calc(self.phase,            table, amp, arr)
            self.v=self.calc((self.phase+0x55555555)&0xFFFFFFFF, table, amp, arr)
            self.w=self.calc((self.phase+0xAAAAAAAA)&0xFFFFFFFF, table, amp, arr)
            self.phase=(self.phase+step)&0xFFFFFFFF

# ---------- timer + top (mirror of bldc_pwm_timer.v / top mux) ----------
class Timer:
    def __init__(self):
        self.psc_cnt=0; self.cnt=0; self.dir=0; self.uev=0
        self.s=[0]*6
        self.dt=[DTGen(),DTGen(),DTGen()]
    def step(self, en, cms, moe, preload, indep, psc, arr,
             ocr, dtg, brk):
        # ----- combinational inputs (from current regs) -----
        tick = (self.psc_cnt==psc)
        cmp = [1 if self.cnt < self.s[i] else 0 for i in range(6)]
        out_en = moe and (not brk)
        ref = [cmp[0], cmp[2], cmp[4]]
        dt_next=[self.dt[i].step(out_en, ref[i], dtg[i]) for i in range(3)]
        # ----- next state of counter / prescaler -----
        npsc = 0 if (not en or tick) else self.psc_cnt+1
        ncnt, ndir, nuev = self.cnt, self.dir, 0
        if not en:
            ncnt, ndir = 0,0
        elif tick:
            if not cms:
                if self.cnt>=arr: ncnt, nuev = 0,1
                else: ncnt=self.cnt+1
            else:
                if not self.dir:
                    if self.cnt>=arr: ndir,ncnt=1,self.cnt-1
                    else: ncnt=self.cnt+1
                else:
                    if self.cnt==0: ndir,ncnt,nuev=0,self.cnt+1,1
                    else: ncnt=self.cnt-1
        # ----- shadow update -----
        ns=list(self.s)
        if (not preload) or nuev:   # transparent or latch at UEV
            ns=list(ocr)
        # ----- outputs (combinational from committed dt regs) -----
        # commit
        self.psc_cnt=npsc; self.cnt=ncnt; self.dir=ndir; self.uev=nuev; self.s=ns
        for i in range(3): self.dt[i].commit(dt_next[i])
        uh,ul=self.dt[0].oh,self.dt[0].ol
        vh,vl=self.dt[1].oh,self.dt[1].ol
        wh,wl=self.dt[2].oh,self.dt[2].ol
        cmpl=[uh,ul,vh,vl,wh,wl]
        indep_out=[cmp[0],cmp[1],cmp[2],cmp[3],cmp[4],cmp[5]]
        raw = indep_out if indep else cmpl
        pwm=[ (raw[i] if out_en else 0) for i in range(6)]
        return pwm

class Top:
    def __init__(self):
        self.timer=Timer(); self.inj=Injector()
    def step(self, cfg, brk):
        # OCR source mux
        if cfg['auto']:
            self.inj.step(True, self.timer.uev, cfg['svpwm'], cfg['sfreq'], cfg['amp'], cfg['arr'])
            ocr=[self.inj.u, cfg['ocr'][1], self.inj.v, cfg['ocr'][3], self.inj.w, cfg['ocr'][5]]
        else:
            self.inj.step(False, 0,0,0,0,cfg['arr'])
            ocr=list(cfg['ocr'])
        pwm=self.timer.step(cfg['en'],cfg['cms'],cfg['moe'],cfg['preload'],cfg['indep'],
                            cfg['psc'],cfg['arr'],ocr,cfg['dtg'],brk)
        return pwm, list(ocr)

# =====================================================================
#  TEST 1 : manual complementary PWM + dead-time, no shoot-through
# =====================================================================
def test1():
    top=Top()
    cfg=dict(en=1,cms=0,moe=1,preload=1,indep=0,auto=0,svpwm=0,
             psc=0,arr=199,ocr=[60,0,100,0,140,0],dtg=[10,10,10],
             sfreq=0,amp=0)
    DT=10
    rec=[]
    for t in range(1200):
        pwm,_=top.step(cfg,0)
        rec.append(pwm)
    rec=np.array(rec)
    # no shoot-through on any phase
    shoot = int(np.sum(rec[:,0]&rec[:,1]) + np.sum(rec[:,2]&rec[:,3]) + np.sum(rec[:,4]&rec[:,5]))
    # measure dead-time on phase U: gap between U_low falling region... measure
    # the number of cycles where both U gates are low at a switching edge.
    uh,ul=rec[:,0],rec[:,1]
    both_low=((uh==0)&(ul==0)).astype(int)
    # find runs of both_low (steady ON periods are not both-low; only DT windows are)
    gaps=[]
    run=0
    for b in both_low[200:1000]:
        if b: run+=1
        elif run>0: gaps.append(run); run=0
    measured_dt = min(gaps) if gaps else 0
    # duty of phase U high side over one steady period (after warmup)
    period=200
    seg=uh[600:600+period]
    duty=seg.sum()/period
    print("TEST1 manual complementary + dead-time")
    print("  shoot-through cycles      : %d   (PASS expect 0)"%shoot)
    print("  measured dead-time (cyc)  : %d   (~ DT+1 = %d)"%(measured_dt, DT+1))
    print("  phase-U high duty          : %.3f (OCR0/ARR=%.3f minus DT)"%(duty,60/200))
    # plot zoom
    plt.figure(figsize=(10,5))
    sl=slice(560,760)
    names=['U_high','U_low','V_high','V_low','W_high','W_low']
    for i in range(6):
        plt.step(np.arange(560,760), rec[sl,i]*0.8+ (5-i)*1.1, where='post')
        plt.text(560, (5-i)*1.1+0.1, names[i], fontsize=8, va='bottom')
    plt.title("Manual complementary PWM with 10-cycle dead-time (no shoot-through)")
    plt.xlabel("clk cycle"); plt.yticks([]); plt.tight_layout()
    plt.savefig("wave_complementary_deadtime.png", dpi=110)
    plt.close()
    return shoot==0 and abs(measured_dt-(DT+1))<=1

# =====================================================================
#  TEST 2 : BREAK forces all outputs off
# =====================================================================
def test2():
    top=Top()
    cfg=dict(en=1,cms=0,moe=1,preload=1,indep=0,auto=0,svpwm=0,
             psc=0,arr=199,ocr=[60,0,100,0,140,0],dtg=[10,10,10],sfreq=0,amp=0)
    for t in range(400): top.step(cfg,0)
    viol=0
    for t in range(20):
        pwm,_=top.step(cfg,1)   # break asserted
        if any(pwm): viol+=1
    print("TEST2 BREAK")
    print("  output-active cycles during break: %d   (PASS expect 0)"%viol)
    return viol==0

# ---- exact-frequency harmonic projection (no spectral leakage) ----
def project(x, step, harm=1):
    """Return (magnitude, phase_deg) of harmonic `harm` of the fundamental
    whose per-sample angle is 2*pi*step/2^32. Exact for the known frequency."""
    k = np.arange(len(x))
    th = 2*math.pi*harm*((k*step) % (2**32))/2**32
    re = np.sum(x*np.cos(th)); im = np.sum(x*np.sin(th))
    return math.hypot(re, im), math.degrees(math.atan2(im, re))

def run_inject(svpwm_sel, arr=255, amp=80, step=67108864, nclk=260000):
    top=Top()
    cfg=dict(en=1,cms=1,moe=1,preload=1,indep=0,auto=1,svpwm=svpwm_sel,
             psc=0,arr=arr,ocr=[0]*6,dtg=[8,8,8],sfreq=step,amp=amp)
    ou=[];ov=[];ow=[]
    for t in range(nclk):
        pwm,ocr=top.step(cfg,0)
        if top.timer.uev:
            ou.append(ocr[0]);ov.append(ocr[2]);ow.append(ocr[4])
    return (np.array(ou,float),np.array(ov,float),np.array(ow,float),
            2**32/step)

# =====================================================================
#  TEST 3 : auto sine injection -> 3-phase sinusoid 120 deg apart
# =====================================================================
def test3():
    step=67108864
    ou,ov,ow,Tsamp=run_inject(svpwm_sel=0, step=step)
    u=ou-ou.mean(); v=ov-ov.mean(); w=ow-ow.mean()
    _,pu=project(u,step); _,pv=project(v,step); _,pw=project(w,step)
    d_uv=(pv-pu)%360
    d_uw=(pw-pu)%360
    def dist120(x): return min(abs(((x-120+180)%360)-180), abs(((x-240+180)%360)-180))
    print("TEST3 auto sine injection (3-phase)")
    print("  elec period (UEV samples) : %.1f, captured %d samples"%(Tsamp,len(ou)))
    print("  phase separations (deg)   : V-U=%.1f, W-U=%.1f  (expect 120 & 240 in some order)"%(d_uv,d_uw))
    n=int(Tsamp*3)
    plt.figure(figsize=(10,4.5))
    plt.plot(ou[:n],label='OCR_U'); plt.plot(ov[:n],label='OCR_V'); plt.plot(ow[:n],label='OCR_W')
    plt.axhline(ou.mean(),color='k',ls='--',lw=0.6)
    plt.title("Auto-injected 3-phase sine compare values (120 deg apart)")
    plt.xlabel("update event #"); plt.ylabel("OCR (0..ARR)"); plt.legend(loc='upper right')
    plt.tight_layout(); plt.savefig("wave_sine_3phase.png", dpi=110); plt.close()
    return dist120(d_uv)<5 and dist120(d_uw)<5

# =====================================================================
#  TEST 4 : SVPWM saddle shape + 3rd-harmonic cancels line-to-line
# =====================================================================
def test4():
    step=67108864
    ou,ov,ow,Tsamp=run_inject(svpwm_sel=1, amp=110, step=step)
    uph=ou-ou.mean()
    uv=(ou-ov); uv=uv-uv.mean()
    f1_ph,_ = project(uph,step,1); f3_ph,_ = project(uph,step,3)
    f1_ll,_ = project(uv ,step,1); f3_ll,_ = project(uv ,step,3)
    r_ph = f3_ph/f1_ph; r_ll = f3_ll/f1_ll
    print("TEST4 SVPWM")
    print("  per-phase 3rd-harm / fund : %.3f  (SVPWM injects 3rd harmonic)"%r_ph)
    print("  line-line 3rd-harm / fund : %.3f  (cancels -> clean sine)"%r_ll)
    n=int(Tsamp*2)
    plt.figure(figsize=(10,4.5))
    plt.plot(ou[:n],label='OCR_U (SVPWM saddle)')
    plt.plot((ou-ov)[:n]+ou.mean(),label='U-V line (sinusoidal)',ls='--')
    plt.axhline(ou.mean(),color='k',ls=':',lw=0.6)
    plt.title("SVPWM: per-phase saddle vs. sinusoidal line-to-line")
    plt.xlabel("update event #"); plt.ylabel("compare value"); plt.legend(loc='upper right')
    plt.tight_layout(); plt.savefig("wave_svpwm.png", dpi=110); plt.close()
    return (r_ph > 0.05) and (r_ll < 0.02)

# =====================================================================
#  Frequency / RPM relationship sanity (formula check)
# =====================================================================
def test5():
    fclk=16e6
    for (arr,psc) in [(800,0),(1000,1)]:
        f_uev = fclk/(2*arr*(psc+1))   # center-aligned
        for f_e in [1,5,10]:
            step=round(f_e/f_uev*2**32)
            f_e_meas = f_uev*step/2**32
            rpm = f_e_meas*60   # 1 pole-pair assumption; *60 for rev/min, /pp
            print("  arr=%d psc=%d f_uev=%.0fHz  target f_e=%dHz -> step=%d -> f_e=%.3fHz (~%.0f RPM, 1 pp)"
                  %(arr,psc,f_uev,f_e,step,f_e_meas,rpm))
    return True

if __name__=="__main__":
    r=[]
    r.append(("complementary+deadtime",test1()))
    r.append(("break",test2()))
    r.append(("sine 3-phase",test3()))
    r.append(("svpwm",test4()))
    print("\nFrequency/RPM relationship (f_e = f_uev * step / 2^32):")
    test5()
    print("\n==== SUMMARY ====")
    for name,ok in r:
        print("  %-24s : %s"%(name,"PASS" if ok else "FAIL"))
