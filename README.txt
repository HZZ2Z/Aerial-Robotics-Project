# Trajectory tracking for a quadrotor in wind

A MATLAB/Simulink implementation of a 62-second inspection flight with take-off, two figure-eight laps and landing. A 16-state integral LQR uses four limited actuator-input integrators and exact Euler-angle torque mapping. A nine-state Kalman filter uses RTK GNSS and measured attitude, with separate gains for normal and degraded GNSS.

![Reference, true and estimated trajectory in wind](report/figs/traj3d.png)

[Six-page report](report/main.pdf) | [3D animation](results/full_20261126_162018/mission_anim.mp4) | [Experiment metrics](results/full_20261126_162018/metrics.json)

## Results

| Estimated feedback | 3D tracking RMS | Peak error |
| --- | ---: | ---: |
| Nominal | 0.026 m | 0.067 m |
| Wind and gusts | 0.119 m | 0.301 m |
| Five seconds of RTK degradation | 0.174 m | 0.685 m |

All 10 required cases met the thrust, torque and 0.61 rad tilt limits. The largest tilt was 26.8 degrees. In the bonus test with five seconds of complete GNSS loss, the recovery peak was 43.4 degrees.

## Run locally

Use MATLAB R2025b, Simulink, Stateflow and Control System Toolbox. Place quad_arch.slx, mission_reference.m and euler_R.m from your instructor's student kit in simulink/student_kit/. Open the repository folder in MATLAB and run:

    main('quick')   % Design checks and two representative cases
    main('full')    % All experiments, report figures and the animation

The simulation builds the completed model at simulink/quad_arch.slx. Each invocation writes to a new folder under results/. Compile report/main.tex after a full run to rebuild the paper.

## Read the code

| File | Responsibility |
| --- | --- |
| [main.m](main.m) | Design, model and core cases, supplementary experiments, report outputs |
| [matlab/params.m](matlab/params.m) | Vehicle, sensor, controller and filter parameters |
| [matlab/course_d_law.m](matlab/course_d_law.m) | Velocity-form controller and input-integrator rates |
| [matlab/course_filter.m](matlab/course_filter.m) | Nine-state observer and GNSS mode switching |
| [matlab/experiments/run_scenarios.m](matlab/experiments/run_scenarios.m) | Shared matrix of core cases |
| [matlab/experiments/run_experiments.m](matlab/experiments/run_experiments.m) | Controller comparisons, bonus trials and repeatability checks |

The earlier 15-state showcase is available at [commit 89825ee](https://github.com/HZZ2Z/Aerial-Robotics-Project/tree/89825ee9e7347e0724117587b56935179aafd386).
