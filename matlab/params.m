function P = params()
%PARAMS All vehicle, sensor, wind, mission, controller and filter parameters.
% Single source of truth; used by every design script and the Simulink models.

% --- Vehicle (project statement) ---
P.m   = 1.2;                       % mass [kg]
P.J   = diag([0.015 0.015 0.026]); % inertia [kg m^2]
P.g   = 9.81;                      % gravity [m/s^2]
P.Tmax    = 24;                    % thrust limit [N]
P.nmaxxy  = 0.5;                   % roll/pitch torque limit [N m]
P.nmaxz   = 0.1;                   % yaw torque limit [N m]
P.tiltmax = 35*pi/180;             % max tilt [rad]
P.kd  = 0.25;                      % linear drag coefficient [N s/m]
P.Ttrim = P.m*P.g;                 % hover thrust [N]

% --- Wind (course: 2 m/s mean along x + OU gusts, sigma=1 m/s, tau=2 s) ---
P.wind.mode  = 'mean_gust';        % none | mean_only | mean_gust | gust_only (component tests only)
P.wind.vmean = 2;                  % mean wind along x [m/s]
P.wind.t0    = 15;                 % mean wind start time [s]
P.wind.tau   = 2;                  % gust filter time constant [s]
P.wind.sigma = 1;                  % gust std per axis [m/s]

% --- Sensors: rate, 1-sigma noise, spectral density r = sigma^2/rate ---
% Course model: continuous true signal + band-limited white noise of density
% r; the noise is held constant over a grid dt (integer multiple of sim.dt).
P.sens.model = 'ct_white_approx';  % 'sample_hold' = legacy extension (sensor_sfun.m)
P.sens.ahrs.rate = 200;  P.sens.ahrs.sigma = 0.5*pi/180;          % rad
P.sens.gyro.rate = 200;  P.sens.gyro.sigma = 0.01;                % rad/s
P.sens.gnss.rate = 50;
P.sens.gnss.sigma = [0.02; 0.02; 0.04];        % RTK fix, h/h/v [m]
P.sens.gnss.bad   = [0.5;  0.5;  1.0];         % RTK fix lost [m]
for f = {'ahrs','gyro','gnss'}
    s = P.sens.(f{1});  s.r = s.sigma.^2 / s.rate;  P.sens.(f{1}) = s;
end
% per-axis spectral densities
P.sens.gnss.rvec  = P.sens.gnss.sigma.^2 / P.sens.gnss.rate;
P.sens.gnss.rbad  = P.sens.gnss.bad.^2   / P.sens.gnss.rate;

% --- Noise generation: root seed + fixed per-stream offsets; hold grids ---
P.noise.seed_base = 1729;          % dev seed; holdout seeds: 1009 2027 4093 8191 16381
P.noise.dt_wind   = 1e-2;          % gust noise update grid [s]
P.noise.dt_sensor = 5e-3;          % AHRS/gyro noise update grid [s]
P.noise.dt_gnss   = 2e-2;          % GNSS noise update grid [s]
P.noise.seed_wind = 21;            % per-axis offsets: base + seed_wind + (0:2)
P.noise.seed_ahrs = 31;
P.noise.seed_gyro = 41;
P.noise.seed_gnss = 51;            % good-RTK stream; degraded stream uses +10

% --- Mission (NED: z down, altitude h -> z* = -h) ---
P.mis.h     = 2;        % cruise altitude [m]
P.mis.tup   = 5;        % climb time [s]
P.mis.thov  = 3;        % hover durations [s]
P.mis.a     = 3;        % lemniscate amplitude [m]
P.mis.T8    = 20;       % figure-eight period [s]
P.mis.laps  = 2;        % number of laps
P.mis.tland = 5;        % landing time [s]
P.mis.tb    = 3;        % polynomial blending window [s]
P.mis.mode  = 'mission';           % mission | q_maneuver | hover_validation
P.mis.t_hv  = 45;                  % hover_validation end time [s]

% --- Simulation ---
% climb + hover + blend-in + 2 laps + blend-out + hover + land
P.sim.tfinal = P.mis.tup + P.mis.thov + P.mis.tb + P.mis.laps*P.mis.T8 ...
             + P.mis.tb + P.mis.thov + P.mis.tland;   % = 60 s
P.sim.dt     = 1e-3;                     % fixed solver step [s]

% --- LQR (Bryson maxima; R scaled by rho) ---
P.lqr.ximax  = 0.5;          % integral-error scale [m s]
P.lqr.xi_max = 0.25;         % integrator clamp [m s]: bounds outage windup (drift
                             % integrates freely otherwise); ~3x the legit wind
                             % integral so plain-wind tracking is unaffected
P.lqr.emax   = 1.0;          % position error scale [m]
P.lqr.vmax   = 1.0;          % velocity error scale [m/s]
P.lqr.etamax = 20*pi/180;    % attitude error scale [rad]
P.lqr.wmax   = 1.0;          % angular rate error [rad/s]
P.lqr.dTmax  = 6;            % thrust deviation scale [N]
P.lqr.nmax   = [0.5; 0.5; 0.1];
P.lqr.rhos   = [0.1 1 10];   % the three required weightings

% --- Kalman filter (tunable; set in kf_design) ---
P.kf.mode          = 'ou_bias12';  % course baseline 'rw9'; ou_bias12 = documented extension
P.kf.qa            = 0.5;    % disturbance-of-acceleration intensity scale
P.kf.qd            = 0.3;    % rw9 random-walk disturbance intensity
% qa/qd flight tuning is constraint-driven: at qa=1/qd=1 the estimated RTK-outage
% transient amplifies the degraded-mode velocity-estimate noise through the LQR
% (tilt 50.6 deg > 35 deg limit); qa=0.5/qd=0.3 keeps every core scenario inside
% the limit (max 32.3 deg) at negligible tracking cost (test_kf sweep is flat).
P.kf.qf            = 0.75;   % ou_bias12 gust (OU) intensity, tau = P.wind.tau
P.kf.qs            = 0.2;    % ou_bias12 slow bias (random walk) intensity
P.kf.reacq_blend_s = 3.0;    % 0: hard gain switch (course baseline); >0: soft re-acq [s]
end
