function out = run_closed(P, K, F, mdl, opts)
%RUN_CLOSED Run closed_loop.slx once for a scenario and collect the logs.
% opts fields (all optional):
%   wind_mode  'none'|'mean_only'|'mean_gust'|'gust_only' (default 'none');
%              legacy scalar wind 0/1 maps to none/mean_gust
%   use_est    via F.use_est: 0 ideal states, 1 estimated states
%   fix_loss   [t0 t1] GNSS outage window [s), fix_drop: degraded | no_fix
%   no_fix     0/1 remove GNSS updates entirely (default 0)
%   m_scale    plant mass scale   (default 1)
%   J_scale    plant inertia scale (default 1)
%   in_delay   plant input delay [s] (default 0)
%   seed       noise root seed (default P.noise.seed_base)
%   dt         solver step (default P.sim.dt)
%   tfinal     stop time (default P.sim.tfinal)
%   tag        scratch log tag (results/scratch/)
%   execution_id / case_id  batch layout (results/<execution_id>/<case_id>/)
% Logs the ACTUAL signals: vw applied to the plant, delayed and saturated
% inputs, the thrust the KF actually used, and derives the disturbance
% truth d_model = a_true - ua_ideal (see the log contract in
% docs/optimization-plan/06_FILE_CHANGE_MAP.md).
if nargin < 5, opts = struct(); end
o = setdef(opts, 'wind_mode', 'none', 'fix_loss', [1e6 1e6+5], 'no_fix', 0, ...
    'm_scale', 1, 'J_scale', 1, 'in_delay', 0, 'seed', P.noise.seed_base, ...
    'dt', P.sim.dt, 'tfinal', P.sim.tfinal, 'tag', '', ...
    'execution_id', '', 'case_id', '');
if isnumeric(o.wind_mode)   % legacy 0/1
    if o.wind_mode == 1, o.wind_mode = 'mean_gust'; else, o.wind_mode = 'none'; end
end

root = fileparts(fileparts(mfilename('fullpath')));
if ~bdIsLoaded('closed_loop')
    load_system(fullfile(root, 'simulink', 'closed_loop.slx'));
end

% base-workspace variables (model resolves from base)
Pp = P; Pp.m = P.m*o.m_scale; Pp.J = P.J*o.J_scale;
switch o.wind_mode
    case 'none',      wind_gain = 0; gust_gain = 0;
    case 'mean_only', wind_gain = 1; gust_gain = 0;
    case 'mean_gust', wind_gain = 1; gust_gain = 1;
    case 'gust_only', wind_gain = 0; gust_gain = 1;
    otherwise, error('run_closed: unknown wind_mode %s', o.wind_mode);
end
Pn = P; Pn.noise.seed_base = o.seed;      % reseed all noise streams
assignin('base', 'Lg', mdl.Lg);     assignin('base', 'Lb', mdl.Lb);
assignin('base', 'Pg', mdl.Pg);     assignin('base', 'Pb', mdl.Pb);
assignin('base', 'P',  Pn);
assignin('base', 'Pp', Pp);
assignin('base', 'K',  K);
assignin('base', 'F',  F);
assignin('base', 'kfmdl', kf_runtime(mdl));
assignin('base', 'wind_gain', wind_gain);
assignin('base', 'gust_gain', gust_gain);
assignin('base', 'fix_loss_t0', o.fix_loss(1));
assignin('base', 'fix_loss_t1', o.fix_loss(2));
assignin('base', 'fix_drop', 1 + o.no_fix);   % 1: degraded fix, 2: no fix at all
assignin('base', 'in_delay', o.in_delay);
set_param('closed_loop', 'FixedStep', num2str(o.dt), 'StopTime', num2str(o.tfinal));

so = sim('closed_loop');
t  = so.log_ref.Time;
out = struct('t', t, ...
    'ref',    so.log_ref.Data,        'p',  so.plant_p.Data, ...
    'v',      so.plant_v.Data,        'eta', so.plant_eta.Data, ...
    'omega',  so.plant_omega.Data,    'R',  so.plant_R.Data, ...
    'vw',     so.wind_vw.Data,        'fix', so.gnss_fix.Data, ...
    'p_meas', so.log_p_meas.Data,     'kf',  so.log_kf_out.Data, ...
    'x15',    so.log_x15.Data,        'ucmd', so.log_ucmd.Data, ...
    'u_delayed', so.log_u_delayed.Data, 'T_model', so.log_T_model.Data, ...
    'eta_meas', so.log_eta_meas.Data, 'p_est', so.log_p_est.Data);
out.opts = o;  out.P = P;  out.Pp = Pp;  out.mdl = rmfields_runtime(mdl);

% --- actual applied input: delay -> saturation (same semantics as the plant)
out.Tn = sat_input(out.u_delayed, Pp);
% --- disturbance truth on the logged grid (all from one dynamics formula) ---
m_t = Pp.m;  m0 = P.m;
Re3   = out.R(:, [3 6 9]);                       % R_t * e3 per row (col-major log)
Rmeas = rot3_measured(out.eta_meas);             % R(eta_meas) * e3 per row
a_true   = -(out.Tn(:,1)/m_t).*Re3 + P.g*[0 0 1] - (P.kd/m_t).*(out.v - out.vw);
ua_ideal = -(out.T_model/m0).*Re3 + P.g*[0 0 1]; % no AHRS noise, nominal mass
ua_meas  = -(out.T_model/m0).*Rmeas + P.g*[0 0 1];
out.a_true = a_true;
out.ua_ideal = ua_ideal;
out.ua_meas = ua_meas;
out.d_model = a_true - ua_ideal;                 % evaluation truth for d_hat
out.d_input_noise = ua_ideal - ua_meas;          % AHRS-induced input error
out.d_effective = a_true - ua_meas;              % actual prediction residual
out.d_wind = -(P.kd/m_t).*(out.v - out.vw);      % wind drag component only
% closed-loop identity residual (numerical diagnostic)
out.resid = max(abs(a_true - (ua_meas + out.d_model + out.d_input_noise)), [], 'all');

% model-predicted error covariance P9(t) from the ACTUAL gain schedule
out.P_out = propagate_covariance(out, mdl);

if ~isempty(o.execution_id)
    d = fullfile(root, 'results', o.execution_id, o.case_id);
    if exist(d, 'dir') ~= 7, mkdir(d); end
    assert(~exist(fullfile(d, 'run.mat'), 'file'), ...
        'run_id collision: %s/%s already has a run', o.execution_id, o.case_id);
    save(fullfile(d, 'run.mat'), 'out');
elseif ~isempty(o.tag)
    d = fullfile(root, 'results', 'scratch');
    if exist(d, 'dir') ~= 7, mkdir(d); end
    save(fullfile(d, ['run_' o.tag '.mat']), 'out');
end
end

function M = kf_runtime(mdl)
% runtime filter struct for kf_sfun (no base-workspace L/P name clashes)
M = struct('nx', mdl.nx, 'tau', mdl.tau, 'm', mdl.m, 'g', mdl.g, ...
    'A', mdl.A, 'B', mdl.B, 'C', mdl.C, 'Lg', mdl.Lg, 'Lb', mdl.Lb, ...
    'Pg', mdl.Pg, 'Pb', mdl.Pb, 'blend_s', mdl.blend_s);
end

function m = rmfields_runtime(mdl)
% slim copy stored inside each run (poles/eig fields are design-time only)
keep = {'mode','nx','tau','A','B','C','Tc','Q','Rg','Rb','Lg','Lb','Pg','Pb', ...
        'blend_s','m','g','q'};
m = struct();
for f = keep
    assert(isfield(mdl, f{1}), 'kf_design output missing field %s', f{1});
    m.(f{1}) = mdl.(f{1});
end
end

function u = sat_input(u, Pp)
T = min(max(u(:,1), 0), Pp.Tmax);
nl = [Pp.nmaxxy; Pp.nmaxxy; Pp.nmaxz];
u = [T, min(max(u(:,2:4), -nl'), nl')];
end

function Rc = rot3_measured(eta)
% R(eta)*e3 rows from Euler angles without a per-row rotation build
sf = sin(eta(:,1)); cf = cos(eta(:,1));
st = sin(eta(:,2)); ct = cos(eta(:,2));
sp = sin(eta(:,3)); cp = cos(eta(:,3));
Rc = [cp.*st.*cf + sp.*sf, sp.*st.*cf - cp.*sf, ct.*cf];
end

function s = setdef(s, varargin)
for k = 1:2:numel(varargin)
    if ~isfield(s, varargin{k}), s.(varargin{k}) = varargin{k+1}; end
end
end
