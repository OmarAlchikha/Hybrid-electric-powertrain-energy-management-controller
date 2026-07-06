#!/usr/bin/env python3
"""Reference co-simulation of the series-hybrid powertrain + supervisor.

Purpose
-------
This script is a line-for-line port of the plant physics and of
``matlab/power_split_controller.m`` (rules R1-R6). It exists so the
energy-management design can be executed and regression-checked in any
environment without a MATLAB license, and so the Simulink model has an
independent implementation to cross-check against (same mission, same
parameters, same rule set -> the two must agree).

If you change a rule in the MATLAB controller, change it here too - the
docstring of ``Supervisor.step`` maps each block of code to its rule tag.

Run:  python3 simulate_hybrid.py
Outputs: ../results/mission_python.png, ../results/genset_histogram.png
plus pass/fail feasibility checks on stdout (non-zero exit on FAIL).
"""

import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULTS = Path(__file__).resolve().parent.parent / "results"


# ----------------------------------------------------------------------
# Parameters: mirror of matlab/hybrid_params.m. KEEP IN SYNC BY HAND.
# All [STAND-IN] tags there apply here identically.
# ----------------------------------------------------------------------
@dataclass
class Params:
    # cell (Project 3 interface, stand-in)
    cell_Q_Ah: float = 2.5
    cell_R0: float = 0.025
    cell_V_min: float = 2.80
    soc_brk: np.ndarray = field(default_factory=lambda: np.arange(0, 1.01, 0.1))
    ocv_V: np.ndarray = field(default_factory=lambda: np.array(
        [3.00, 3.30, 3.43, 3.49, 3.53, 3.58, 3.64, 3.71, 3.80, 3.94, 4.15]))
    # pack
    Ns: int = 96
    Np: int = 15
    soc0: float = 0.90
    P_dis_max: float = 90e3
    P_chg_max: float = 18e3
    # drive (Project 4 interface, stand-in)
    drive_P_peak: float = 70e3
    drive_eta: float = 0.92
    drive_eta_regen: float = 0.85
    # genset (stand-in; rated so cruise sits in the best-BSFC band)
    gen_P_rated: float = 40e3
    gen_P_min: float = 8e3
    gen_eff_lo: float = 0.60
    gen_eff_hi: float = 0.90
    gen_rate_W_s: float = 5e3
    gen_tau: float = 3.0
    # controller
    Ts: float = 0.1
    soc_lo: float = 0.45
    soc_hi: float = 0.75
    soc_target: float = 0.65
    t_min_on: float = 60.0
    t_min_off: float = 60.0
    k_chg: float = 40e3
    P_dem_on: float = 40e3
    # sim
    Ts_plant: float = 0.01
    t_end: float = 1800.0

    @property
    def pack_Q_Ah(self):
        return self.Np * self.cell_Q_Ah

    @property
    def pack_R0(self):
        return self.cell_R0 * self.Ns / self.Np

    def pack_ocv(self, soc):
        return self.Ns * np.interp(soc, self.soc_brk, self.ocv_V)


# ----------------------------------------------------------------------
# Mission: mirror of matlab/mission_profile.m (shaft power, W)
# ----------------------------------------------------------------------
SEG_T = np.array([0, 120, 180, 480, 1500, 1560, 1680, 1800.0])
SEG_P = np.array([6e3, 65e3, 48e3, 30e3, -5e3, 4e3, 6e3, 6e3])
RAMP = 5.0


def mission_shaft_power(t):
    idx = int(np.searchsorted(SEG_T, t, side="right") - 1)
    p = SEG_P[idx]
    if idx > 0 and (t - SEG_T[idx]) < RAMP:
        a = (t - SEG_T[idx]) / RAMP
        p = (1 - a) * SEG_P[idx - 1] + a * SEG_P[idx]
    return p


def drive_interface(p_shaft, P):
    """Mirror of matlab/drive_interface.m."""
    p_shaft = min(p_shaft, P.drive_P_peak)
    if p_shaft >= 0:
        return p_shaft / P.drive_eta
    return p_shaft * P.drive_eta_regen


# ----------------------------------------------------------------------
# Supervisor: port of matlab/power_split_controller.m
# ----------------------------------------------------------------------
class Supervisor:
    def __init__(self, P: Params):
        self.P = P
        self.gen_on = 0
        self.t_switch = -P.t_min_off
        self.P_gen_prev = 0.0
        self.t_now = -P.Ts  # first step lands on t=0, like the persistent init

    def step(self, P_dem_elec, soc, P_gen_meas):
        P = self.P
        self.t_now += P.Ts

        # R1: SOC-tapered battery limits
        taper_hi = min(1.0, max(0.0, (0.95 - soc) / 0.05))
        taper_lo = min(1.0, max(0.0, (soc - 0.05) / 0.05))
        P_dis_lim = P.P_dis_max * taper_lo
        P_chg_lim = P.P_chg_max * taper_hi

        # R2: on/off with hysteresis + dwell
        dwell_ok_on = (self.t_now - self.t_switch) >= P.t_min_off
        dwell_ok_off = (self.t_now - self.t_switch) >= P.t_min_on
        want_on = (soc < P.soc_lo) or (P_dem_elec > P.P_dem_on)
        want_off = (soc > P.soc_hi) and (P_dem_elec < P.gen_P_min)
        if not self.gen_on and want_on and dwell_ok_on:
            self.gen_on, self.t_switch = 1, self.t_now
        elif self.gen_on and want_off and dwell_ok_off:
            self.gen_on, self.t_switch = 0, self.t_now

        # R3 + R4: power target
        if self.gen_on:
            P_chg_bias = P.k_chg * (P.soc_target - soc)
            P_chg_bias = max(min(P_chg_bias, P_chg_lim), 0.0)
            P_tgt = P_dem_elec + P_chg_bias
            P_tgt = min(max(P_tgt, P.gen_P_min), P.gen_P_rated)
            P_band_lo = P.gen_eff_lo * P.gen_P_rated
            if P_tgt < P_band_lo:
                surplus_cap = P_chg_lim - max(P_tgt - P_dem_elec, 0.0)
                P_tgt = min(P_band_lo, P_tgt + max(surplus_cap, 0.0))
            if P_dem_elec < 0:
                P_tgt = P.gen_P_min
        else:
            P_tgt = 0.0

        # R5: slew limit
        dP = P.gen_rate_W_s * P.Ts
        P_gen_cmd = min(max(P_tgt, self.P_gen_prev - dP), self.P_gen_prev + dP)
        P_gen_cmd = max(P_gen_cmd, 0.0)
        self.P_gen_prev = P_gen_cmd

        # R6: battery expectation + feasibility accounting (MEASURED power)
        P_batt_exp = P_dem_elec - P_gen_meas
        P_unmet = P_dump = 0.0
        if P_batt_exp > P_dis_lim:
            P_unmet = P_batt_exp - P_dis_lim
            P_batt_exp = P_dis_lim
        elif P_batt_exp < -P_chg_lim:
            P_dump = -(P_batt_exp + P_chg_lim)
            P_batt_exp = -P_chg_lim

        if P_dem_elec < 0:
            mode = 3
        elif not self.gen_on:
            mode = 0
        elif P_batt_exp > 1e2:
            mode = 1
        else:
            mode = 2
        return P_gen_cmd, P_batt_exp, self.gen_on, mode, P_unmet, P_dump


# ----------------------------------------------------------------------
# Plant + simulation loop
# ----------------------------------------------------------------------
def battery_step(P: Params, soc, p_batt, dt):
    """OCV+R0 Thevenin step. p_batt > 0 = discharge (W at terminals).

    Terminal power p = v*i with v = ocv - i*R0 gives
    i = (ocv - sqrt(ocv^2 - 4*R0*p)) / (2*R0)  (physical root).
    """
    ocv = P.pack_ocv(soc)
    r = P.pack_R0
    disc = ocv * ocv - 4.0 * r * p_batt
    if disc < 0:  # demand beyond max power transfer -> should never happen
        raise RuntimeError(f"battery power infeasible: {p_batt/1e3:.1f} kW")
    i = (ocv - np.sqrt(disc)) / (2.0 * r)
    v = ocv - i * r
    soc = soc - i * dt / (3600.0 * P.pack_Q_Ah)
    return float(np.clip(soc, 0.0, 1.0)), v, i


def simulate(P: Params):
    n = int(round(P.t_end / P.Ts)) + 1
    sup = Supervisor(P)
    soc, p_gen_act = P.soc0, 0.0
    log = {k: np.zeros(n) for k in
           ("t", "p_shaft", "p_dem", "p_gen", "p_batt", "soc", "v_bus",
            "i_batt", "mode", "unmet", "dump", "gen_on")}
    sub = int(round(P.Ts / P.Ts_plant))  # plant substeps per supervisor step

    for k in range(n):
        t = k * P.Ts
        p_shaft = mission_shaft_power(t)
        p_dem = drive_interface(p_shaft, P)
        p_gen_cmd, _, gen_on, mode, unmet, dump = sup.step(
            p_dem, soc, p_gen_act)

        # net load actually presented to the bus after curtailment/dump
        p_load = p_dem - unmet + dump
        for _ in range(sub):  # plant at Ts_plant
            p_gen_act += (P.Ts_plant / P.gen_tau) * (p_gen_cmd - p_gen_act)
            soc, v_bus, i_batt = battery_step(
                P, soc, p_load - p_gen_act, P.Ts_plant)

        log["t"][k] = t
        log["p_shaft"][k] = p_shaft
        log["p_dem"][k] = p_dem
        log["p_gen"][k] = p_gen_act
        log["p_batt"][k] = p_load - p_gen_act
        log["soc"][k] = soc
        log["v_bus"][k] = v_bus
        log["i_batt"][k] = i_batt
        log["mode"][k] = mode
        log["unmet"][k] = unmet
        log["dump"][k] = dump
        log["gen_on"][k] = gen_on
    return log


# ----------------------------------------------------------------------
# Feasibility checks (the "tests" of the energy management design)
# ----------------------------------------------------------------------
def check(log, P: Params, recovery=False):
    ok = True

    def req(name, cond, detail):
        nonlocal ok
        print(f"  {'PASS' if cond else 'FAIL'}  {name}: {detail}")
        ok &= cond

    req("demand always met", np.all(log["unmet"] < 1.0),
        f"max unmet {log['unmet'].max():.1f} W")
    req("SOC stays above floor", log["soc"].min() > 0.20,
        f"min SOC {100*log['soc'].min():.1f} %")
    if recovery:
        # low-SOC scenario: charge bias must claw SOC back up
        req("SOC recovers above start", log["soc"][-1] > log["soc"][0] + 0.05,
            f"start {100*log['soc'][0]:.0f} % -> final {100*log['soc'][-1]:.1f} %")
    else:
        req("SOC charge-sustaining-ish", log["soc"][-1] > P.soc_lo,
            f"final SOC {100*log['soc'][-1]:.1f} % (band lo {100*P.soc_lo:.0f} %)")
    req("bus voltage above cutoff", log["v_bus"].min() > P.Ns * P.cell_V_min,
        f"min {log['v_bus'].min():.0f} V vs cutoff {P.Ns*P.cell_V_min:.0f} V")
    req("battery discharge within limit",
        log["p_batt"].max() <= P.P_dis_max * 1.02,
        f"max discharge {log['p_batt'].max()/1e3:.1f} kW / lim {P.P_dis_max/1e3:.0f} kW")
    req("battery charge within limit",
        log["p_batt"].min() >= -P.P_chg_max * 1.02,
        f"max charge {-log['p_batt'].min()/1e3:.1f} kW / lim {P.P_chg_max/1e3:.0f} kW")
    slew = np.abs(np.diff(log["p_gen"])) / P.Ts
    req("genset slew respected", slew.max() <= P.gen_rate_W_s * 1.05,
        f"max {slew.max()/1e3:.2f} kW/s / lim {P.gen_rate_W_s/1e3:.0f} kW/s")
    # no short-cycling: count ON/OFF edges, verify dwell
    edges = np.flatnonzero(np.diff(log["gen_on"]) != 0)
    dwells = np.diff(edges) * P.Ts if len(edges) > 1 else np.array([np.inf])
    req("no genset short-cycling", dwells.min() >= min(P.t_min_on, P.t_min_off),
        f"{len(edges)} switch events, min dwell {dwells.min():.0f} s")
    # energy balance closure: gen + battery == load + dump (integrated)
    e_gen = np.trapezoid(log["p_gen"], log["t"])
    e_load = np.trapezoid(log["p_dem"] - log["unmet"], log["t"])
    e_dump = np.trapezoid(log["dump"], log["t"])
    e_batt = np.trapezoid(log["p_batt"], log["t"])
    resid = abs(e_gen + e_batt - e_load - e_dump) / max(e_load, 1)
    req("energy balance closes", resid < 1e-6,
        f"residual {100*resid:.2e} % of load energy")
    return ok


def report(log, P: Params):
    on = log["gen_on"] > 0.5
    frac_on = on.mean()
    in_band = on & (log["p_gen"] >= P.gen_eff_lo * P.gen_P_rated) \
                 & (log["p_gen"] <= P.gen_eff_hi * P.gen_P_rated)
    e_gen = np.trapezoid(log["p_gen"], log["t"]) / 3.6e6
    e_batt_dis = np.trapezoid(np.maximum(log["p_batt"], 0), log["t"]) / 3.6e6
    print(f"\n  genset ON {100*frac_on:.0f} % of mission; "
          f"while ON, {100*in_band.sum()/max(on.sum(),1):.0f} % of time "
          f"in best-BSFC band ({P.gen_eff_lo:.0%}-{P.gen_eff_hi:.0%} of rated)")
    print(f"  genset energy {e_gen:.2f} kWh, battery throughput "
          f"(discharge) {e_batt_dis:.2f} kWh")
    print(f"  SOC: start {100*log['soc'][0]:.0f} % -> min "
          f"{100*log['soc'].min():.1f} % -> end {100*log['soc'][-1]:.1f} %")


# ----------------------------------------------------------------------
# Plots (panel layout mirrors matlab/plot_results.m)
# ----------------------------------------------------------------------
def make_plots(log, P: Params, suffix=""):
    RESULTS.mkdir(exist_ok=True)
    tmin = log["t"] / 60.0

    fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    ax[0].plot(tmin, log["p_dem"] / 1e3, label="bus demand", lw=1.4)
    ax[0].plot(tmin, log["p_gen"] / 1e3, label="genset", lw=1.4)
    ax[0].plot(tmin, log["p_batt"] / 1e3, label="battery (+ = discharge)", lw=1.4)
    ax[0].axhline(P.gen_P_rated / 1e3, ls=":", c="gray", lw=1)
    ax[0].set_ylabel("kW"); ax[0].legend(loc="upper right", ncol=3)
    ax[0].set_title("Power split (dotted: genset rating)")

    ax[1].plot(tmin, 100 * log["soc"], lw=1.4, c="tab:green")
    ax[1].axhline(100 * P.soc_lo, ls="--", c="gray", lw=1)
    ax[1].axhline(100 * P.soc_hi, ls="--", c="gray", lw=1)
    ax[1].set_ylabel("SOC %")
    ax[1].set_title("Battery SOC (dashed: supervisor hysteresis band)")

    ax[2].plot(tmin, log["v_bus"], lw=1.4, c="tab:purple")
    ax[2].axhline(P.Ns * P.cell_V_min, ls="--", c="tab:red", lw=1)
    ax[2].set_ylabel("V")
    ax[2].set_title("DC bus voltage (dashed: pack cutoff)")

    ax[3].step(tmin, log["mode"], where="post", lw=1.4, c="tab:orange")
    ax[3].set_yticks(range(4))
    ax[3].set_yticklabels(["batt-only", "assist", "recharge", "regen"])
    ax[3].set_xlabel("mission time, min")
    ax[3].set_title("Supervisor mode")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(RESULTS / f"mission_python{suffix}.png", dpi=140)
    if suffix:  # histogram only for the nominal scenario
        plt.close("all")
        return

    # genset operating-point histogram: the visual argument for R4
    fig2, ax2 = plt.subplots(figsize=(8, 4.5))
    on = log["gen_on"] > 0.5
    ax2.hist(100 * log["p_gen"][on] / P.gen_P_rated, bins=40,
             color="tab:blue", alpha=0.85)
    ax2.axvspan(100 * P.gen_eff_lo, 100 * P.gen_eff_hi, color="tab:green",
                alpha=0.15, label="best-BSFC band")
    ax2.set_xlabel("genset load while ON, % of rated")
    ax2.set_ylabel("supervisor steps")
    ax2.set_title("Genset operating points (rule R4 load-levelling)")
    ax2.legend(); ax2.grid(alpha=0.3)
    fig2.tight_layout()
    fig2.savefig(RESULTS / "genset_histogram.png", dpi=140)
    print(f"\n  plots written to {RESULTS}/")


if __name__ == "__main__":
    P = Params()
    print(f"Series-hybrid EMS co-simulation | pack {P.Ns}s{P.Np}p "
          f"{P.pack_ocv(0.5)*P.pack_Q_Ah/1e3:.1f} kWh-class | "
          f"genset {P.gen_P_rated/1e3:.0f} kW | mission {P.t_end/60:.0f} min")

    print("\n=== Scenario 1: nominal mission (SOC0 = 90 %) ===")
    log = simulate(P)
    print("Feasibility checks:")
    ok = check(log, P)
    report(log, P)
    make_plots(log, P)

    # Exercises the branches the nominal mission never reaches: forced-ON
    # below soc_lo, the charge-bias recharge path, and R4 load-levelling
    # during taxi (genset raised into band to charge the pack).
    print("\n=== Scenario 2: low-SOC start (SOC0 = 35 %) ===")
    P2 = Params(soc0=0.35)
    log2 = simulate(P2)
    print("Feasibility checks:")
    ok &= check(log2, P2, recovery=True)
    report(log2, P2)
    make_plots(log2, P2, suffix="_lowsoc")

    sys.exit(0 if ok else 1)
