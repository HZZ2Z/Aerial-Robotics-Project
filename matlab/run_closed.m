function out = run_closed(P, K, F, M, opts)
%RUN_CLOSED Simulate the completed teacher model with scenario-local parameters.
if nargin < 5, opts = struct(); end
o = setdef(opts, 'wind_mode','none','fix_loss',[inf inf],'no_fix',false, ...
    'm_scale',1,'J_scale',1,'in_delay',0,'seed',P.noise.seed_base, ...
    'dt',P.sim.dt,'tfinal',P.sim.tfinal,'execution_id','','case_id','','tag','');
root = fileparts(fileparts(mfilename('fullpath')));
mdl = 'quad_arch';
if ~bdIsLoaded(mdl), build_course_model(P); end
C = course_config(P,K,F,M,o);
REF = course_reference_table(P,o.dt,o.tfinal);
in = Simulink.SimulationInput(mdl);
in = in.setVariable('P',C).setVariable('REF',REF);
so = sim(in);

x = so.log_state.Data;
meas = so.log_meas.Data;
out = struct('t',so.log_state.Time,'ref',so.log_ref.Data, ...
    'p',x(:,1:3),'v',x(:,4:6),'eta',x(:,7:9),'omega',x(:,10:12), ...
    'vw',so.log_wind.Data,'fix',so.log_fix.Data,'p_meas',so.log_p_meas.Data, ...
    'eta_meas',meas(:,1:3),'kf',so.log_kf.Data,'w',so.log_w.Data, ...
    'ucmd',so.log_raw_command.Data,'Tn',so.log_applied.Data, ...
    'T_model',so.log_command.Data(:,1),'opts',o,'P',P,'Pp',P,'mdl',M);
out.Pp.m = C.m_true;
out.Pp.J = C.J_true;
out.p_est = out.kf(:,1:3);
out.u_delayed = out.Tn;
out.R = rotation_rows(out.eta);
Rmeas = rotation_rows(out.eta_meas);
out.d_wind = -P.kd/C.m_true*(out.v-out.vw);
ge3 = [0 0 P.g];
out.a_true = -out.Tn(:,1)/C.m_true.*out.R(:,7:9) + ge3 + out.d_wind;
out.ua_ideal = -out.T_model/P.m.*out.R(:,7:9) + ge3;
out.ua_meas = -out.T_model/P.m.*Rmeas(:,7:9) + ge3;
out.d_model = out.a_true-out.ua_ideal;
out.d_input_noise = out.ua_ideal-out.ua_meas;
out.d_effective = out.a_true-out.ua_meas;
out.resid = max(abs(out.d_effective-out.d_model-out.d_input_noise),[],'all');
assert(all(isfinite(x),'all') && all(isfinite(out.kf),'all') && ...
    abs(out.t(end)-o.tfinal) < o.dt/2, 'Simulation produced an incomplete or invalid log.');
assert(all(out.Tn(:,1) >= -1e-9 & out.Tn(:,1) <= P.Tmax+1e-9) && ...
    all(abs(out.Tn(:,2:4)) <= P.lqr.nmax'+1e-9,'all'), 'Applied actuator limit violated.');
if F.use_est
    out.P_out = propagate_covariance(out,M);
else
    out.P_out = [];
end

if ~isempty(o.execution_id)
    d = fullfile(root,'results',o.execution_id,o.case_id);
    if ~isfolder(d), mkdir(d); end
    target = fullfile(d,'run.mat');
    assert(~isfile(target),'The run already exists: %s',target);
    save(target,'out');
elseif ~isempty(o.tag)
    d = fullfile(root,'results','scratch');
    if ~isfolder(d), mkdir(d); end
    save(fullfile(d,['run_' o.tag '.mat']),'out');
end
end

function R = rotation_rows(a)
c = cos(a); s = sin(a);
cf = c(:,1); ct = c(:,2); cp = c(:,3);
sf = s(:,1); st = s(:,2); sp = s(:,3);
R = [cp.*ct, sp.*ct, -st, ...
     cp.*st.*sf-sp.*cf, sp.*st.*sf+cp.*cf, ct.*sf, ...
     cp.*st.*cf+sp.*sf, sp.*st.*cf-cp.*sf, ct.*cf];
end

function s = setdef(s,varargin)
for k = 1:2:numel(varargin)
    if ~isfield(s,varargin{k}), s.(varargin{k}) = varargin{k+1}; end
end
end
