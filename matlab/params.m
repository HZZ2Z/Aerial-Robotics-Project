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

% --- Wind ---
P.wind.vmean = 2;                  % mean wind along x [m/s]
P.wind.t0    = 15;                 % mean wind start time [s]
P.wind.tau   = 2;                  % gust filter time constant [s]
P.wind.sigma = 1;                  % gust std per axis [m/s]

% --- Sensors: rate, 1-sigma noise, spectral density r = sigma^2*Dt ---
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

% --- Mission (NED: z down, altitude h -> z* = -h) ---
P.mis.h     = 2;        % cruise altitude [m]
P.mis.tup   = 5;        % climb time [s]
P.mis.thov  = 3;        % hover durations [s]
P.mis.a     = 3;        % lemniscate amplitude [m]
P.mis.T8    = 20;       % figure-eight period [s]
P.mis.laps  = 2;        % number of laps
P.mis.tland = 5;        % landing time [s]
P.mis.tb    = 3;        % polynomial blending window [s]
P.mis.man   = 0;        % 1: dedicated Q(phi,theta) test manoeuvre

% --- Simulation ---
% climb + hover + blend-in + 2 laps + blend-out + hover + land
P.sim.tfinal = P.mis.tup + P.mis.thov + P.mis.tb + P.mis.laps*P.mis.T8 ...
             + P.mis.tb + P.mis.thov + P.mis.tland;   % = 60 s
P.sim.dt     = 1e-3;                     % fixed solver step [s]

% --- LQR (Bryson maxima; R scaled by rho) ---
P.lqr.ximax  = 0.5;          % integral-error scale [m s]
P.lqr.emax   = 1.0;          % position error scale [m]
P.lqr.vmax   = 1.0;          % velocity error scale [m/s]
P.lqr.etamax = 20*pi/180;    % attitude error scale [rad]
P.lqr.wmax   = 1.0;          % angular rate error [rad/s]
P.lqr.dTmax  = 6;            % thrust deviation scale [N]
P.lqr.nmax   = [0.5; 0.5; 0.1];
P.lqr.rhos   = [0.1 1 10];   % the three required weightings

% --- Kalman filter (tunable; set in kf_design) ---
P.kf.qa = 1.0;    % disturbance-of-acceleration intensity scale (tuned)
P.kf.qf = 4.0;    % gust (OU) driving intensity, tau = P.wind.tau (4.0 = tilt-constrained empirical optimum; theory match was 0.2)
P.kf.qs = 0.25;    % slow bias (random walk) intensity: mean wind drag, model error
end
