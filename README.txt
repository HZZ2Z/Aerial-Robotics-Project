Aerial Robotics Project 2026 - Trajectory Tracking of a Quadrotor in Wind
LQR Control and Kalman-Filter Navigation with RTK GNSS
==========================================================================

CONTENTS
  main.m                 one-click script: builds models, designs controller
                         and filter, runs all scenarios, regenerates every
                         figure of the report and the 3-D animation
  matlab/                design scripts and Level-2 MATLAB S-functions
    params.m             all vehicle / sensor / wind / mission parameters
    gen_ref.m            C^2 reference mission trajectory (quintic blending)
    lin_hover.m          15-state hover linearisation + self-checks
                         (numeric Jacobian, controllability rank, decoupled chains)
    lqr_design.m         Bryson weights, LQR gains for rho = 0.1/1/10, step metrics
    kf_design.m          9-state steady-state Kalman filter (ARE), observability
    quad_dynamics.m      nonlinear 6-DOF model (NED), saturations included
    quad_sfun.m          plant S-function
    ref_sfun.m           reference generator S-function
    ctrl_sfun.m          LQR + integral action + anti-windup + exact Q(phi,theta)
                         torque mapping + optional disturbance feed-forward
    kf_sfun.m            steady-state Kalman filter S-function (fix-status gains)
    sensor_sfun.m        sampled sensor noise models (AHRS / gyro / GNSS)
    build_plant.m        open-loop plant test model
    build_closed.m       closed_loop.slx (complete controller-estimator loop)
    run_closed.m         run one scenario, collect logs
    run_scenarios.m      the four scenarios x {ideal, estimated} + metrics table
    metrics.m            RMS/max errors, hover offset, estimation errors, tilt, effort
    make_figures.m       all report figures
    animate_mission.m    3-D animation (mp4)
    test_plant.m         open-loop plant verification
    test_closed.m        ideal-state closed-loop verification
    test_kf.m            filter tuning sweep and consistency validation
    test_bonus.m         Q(phi,theta) manoeuvre, feed-forward, GNSS outage
  simulink/
    closed_loop.slx      complete loop (built programmatically by build_closed.m)
    plant_test.slx       open-loop test model

RUN
  1. Open MATLAB (R2023b+ recommended), cd to this folder.
  2. Run:  main
     (~30 min; regenerates results/, report figures and the animation).
  3. Individual tests: test_plant, test_closed, test_kf, test_bonus.

NOTE
  All simulation code runs headless (no Simulink GUI interaction needed);
  open simulink/closed_loop.slx to inspect the wiring.
