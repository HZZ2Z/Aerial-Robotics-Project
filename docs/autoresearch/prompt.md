# Goal

Improve closed-loop tracking of the quadrotor flying on **estimated states**, focusing
on the hardest scenario: 5 s RTK outage during wind (baseline RMS 0.152 m, peak 0.698 m,
31° tilt transient at re-acquisition). Candidate levers: Kalman-filter tuning
(P.kf.qa/qd, gain-switching structure), LQR weights (P.lqr.* Bryson scales, rho),
estimator/controller structure tweaks in kf_sfun.m / ctrl_sfun.m.

# Metric

ertk_rms (lower is better): RMS 3-D tracking error, estimated states, wind + RTK lost
during [20, 25] s. From .auto/measure.sh — deterministic (seeded sensor noise),
single run per experiment, no repeat needed.

# Secondary constraints

- ewind_rms (estimated, plain wind) must not regress: constraint maxPct 110 in
  log_experiment + frozen absolute bound 0.095 in checks.sh.
- tilt < 35° in both scenarios (frozen in checks.sh).

# Files in scope

matlab/params.m, matlab/lqr_design.m, matlab/kf_design.m, matlab/kf_sfun.m,
matlab/ctrl_sfun.m, .auto/prompt.md

# Off limits

.auto/measure.sh, .auto/checks.sh, .auto/bench_ertk.m (frozen benchmark),
matlab/quad_dynamics.m (plant = ground truth), matlab/sensor_sfun.m (sensors are
given by the assignment), matlab/gen_ref.m + ref_sfun.m (mission definition),
matlab/metrics.m (metric definition), matlab/run_*.m + build_*.m (harness),
report/, docs/, results/, simulink/*.slx structure (only rebuild via build_closed
if an sfun signature changes — avoid).

# Protocol note (environment)

The MCP `run_experiment` tool call is killed by a 60 s client timeout — the benchmark
needs ~8 min (MATLAB startup + 2 Simulink runs). The loop therefore executes the SAME
frozen `.auto/measure.sh` via local Bash (identical semantics, deterministic output)
and records results with `log_experiment`; ledger, commit/revert and audit still run
through the plugin. Verified bit-identical values across git-bash and WSL execution.

# What's Been Tried

(baseline) rho=1 LQR + kf qa=qd=1 9-state, hard gain switching: ertk_rms=0.136876,
ertk_max=0.615, tilt 27.3°, ewind_rms=0.0791 — README table numbers came from an
earlier tuning; frozen bench is the source of truth.
(keep, run 3) kf_sfun soft re-acquisition: 1.5 s Lb→Lg gain ramp (extra ts state):
0.136314 (−0.41%). KEY FINDING: peak/tilt bit-identical → transient happens
DURING the outage, not at re-acquisition.
(discard) runs 2/4/5/6: qd<1, qa<1, Lb×0.5 all hurt RMS — 9-state tuning exhausted.
(keep, run 7) 12-state two-time-scale disturbance model in kf_design+kf_sfun
([p;v;df(OU τ=P.wind.tau);ds(RW)], combined 9-dim output, Pg9=T·P12·T′):
0.136220. Physically-correct model; small immediate gain.
(keep, runs 8-11) qf sweep 0.2→0.5→1.0→2.0→4.0, all monotone improvements to
BEST=0.126102 @ qf=4.0 (−7.9% total vs baseline). ewind improved throughout
(0.0791→0.0742). Tilt rises with qf: 27.3°→31.4°.
(checks_failed, runs 12/13/15) qf=8/6/5: tilt 47.8°/49.2°/39.9° > 35° bound AND
RMS worse — sharp saturation cliff between qf=4 and 5. qf=4.0 is the
tilt-constrained optimum (3.6° margin).
(noop, run 14) misfire: sed didn't match after auto-revert, re-ran qf=4.0 config
— bit-identical duplicate, extra determinism proof.
(discard, run 16) revisit of run 5 (qa=0.5 on the new structure): still +0.85%
worse — qa=1.0 confirmed optimal in both structures.

# Segment 1 conclusion (20 runs, cap reached)

best ertk_rms=0.124910 (−8.7% vs 0.136876 baseline), keeps on branch
autoresearch/rtk-outage (d6447f9..f69450c): re-acquisition ramp + 12-state
two-time-scale KF + qf=4.0 (tilt-bound-limited) + qs=0.05 (converged sweep
1.0→0.5→0.25→0.1→0.05, −0.42/−0.28/−0.19/−0.05% per halving). All secondary
metrics improved or flat: ewind 0.0791→0.0745, ertk_max 0.615→0.547, tilt
27.3°→30.1° (still 4.9° under bound). Estimator design space closed:
qa=1, qf=4, qs=0.05, ARE-optimal Lg/Lb, soft re-acquisition ramp.

# Next-segment ideas (untested)

- Fix-aware (certainty-equivalence-breaking) LQR: scale position/integral
  feedback with fix status or Pss diag — needs ctrl_sfun input port + model
  rewiring (charter: avoid unless necessary).
- LQR weight sweep (P.lqr.emax/etamax): low expected value — outage error is
  drift-dominated, not noise-reaction.
- Regenerate README/report tables with the new tuning (ertk 0.152→0.125).
