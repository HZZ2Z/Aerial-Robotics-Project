function out = run_closed(P, K, F, kfdat, opts)
%RUN_CLOSED Run closed_loop.slx once for a scenario and collect the logs.
% opts fields (all optional):
%   wind       0/1 mean wind + gusts                 (default 1)
%   use_est    0 ideal states, 1 estimated states    (default 0)
%   Qapprox    0 exact Q(phi,theta), 1 Q~I           (default 0)
%   ff_wind    0/1 disturbance feed-forward (bonus)  (default 0)
%   fix_loss   [t0 t1] GNSS fix lost window          (default none)
%   no_fix     0/1 remove GNSS updates entirely      (default 0)
%   m_scale    plant mass scale                      (default 1)
%   J_scale    plant inertia scale                   (default 1)
%   in_delay   plant input delay [s]                 (default 0)
%   dt         solver step                           (default P.sim.dt)
%   tfinal     stop time                             (default P.sim.tfinal)
%   tag        log file tag (saved to results/)      (default '')
if nargin < 5, opts = struct(); end
o = setdef(opts, 'wind', 1, 'use_est', 0, 'Qapprox', 0, 'ff_wind', 0, ...
    'fix_loss', [1e6 1e6+5], 'no_fix', 0, 'm_scale', 1, 'J_scale', 1, ...
    'in_delay', 0, 'dt', P.sim.dt, 'tfinal', P.sim.tfinal, 'tag', '');

root = fileparts(fileparts(mfilename('fullpath')));
if ~bdIsLoaded('closed_loop')
    load_system(fullfile(root, 'simulink', 'closed_loop.slx'));
end

% base-workspace variables (model resolves from base)
Pp = P; Pp.m = P.m*o.m_scale; Pp.J = P.J*o.J_scale;
assignin('base', 'Lg', kfdat.Lg);   assignin('base', 'Lb', kfdat.Lb);
assignin('base', 'P',  P);
assignin('base', 'Pp', Pp);
assignin('base', 'K',  K);
assignin('base', 'F',  F);
assignin('base', 'Pg', kfdat.Pg);
assignin('base', 'Pb', kfdat.Pb);
assignin('base', 'wind_gain', o.wind);
assignin('base', 'gust_power', 4);      % calibrated: gust std ~1 m/s
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
    'Tn',     so.plant_Tn.Data, ...
    'vw',     so.wind_vw.Data,        'fix', so.gnss_fix.Data, ...
    'p_meas', so.log_p_meas.Data,     'kf',  so.log_kf_out.Data, ...
    'x15',    so.log_x15.Data,        'ucmd', so.log_ucmd.Data, ...
    'eta_meas', so.log_eta_meas.Data, 'p_est', so.log_p_est.Data);
out.opts = o;
if ~isempty(o.tag)
    save(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', ...
        ['run_' o.tag '.mat']), 'out');
end
end

function s = setdef(s, varargin)
for k = 1:2:numel(varargin)
    if ~isfield(s, varargin{k}), s.(varargin{k}) = varargin{k+1}; end
end
end
