function poles = course_hover_stability(P,K,M)
%COURSE_HOVER_STABILITY Jacobian of all coupled flight/observer states at trim.
opt = struct('m_scale',1,'J_scale',1,'dt',P.sim.dt,'tfinal',P.sim.tfinal, ...
    'no_fix',false,'in_delay',0,'fix_loss',[inf inf],'seed',P.noise.seed_base, ...
    'wind_mode','none');
F = struct('use_est',true,'use_Qapprox',false,'ff_wind',false);
C = course_config(P,K,F,M,opt);
yaw = [0 pi/2 pi];
poles = zeros(31,2,numel(yaw));
for k = 1:numel(yaw)
    x = zeros(31,1);
    x(9) = yaw(k);
    ref = zeros(18,1);
    ref(10) = yaw(k);
    for fix = 0:1
        assert(norm(rate(x,fix,ref,C),inf) < 1e-10,'Invalid hover operating point.');
        A = course_jacobian(@(s) rate(s,fix,ref,C),x);
        poles(:,fix+1,k) = eig(A);
    end
end
disp('Full hover Jacobian: max Re(lambda), rows degraded / RTK, columns heading 0 / 90 / 180 deg');
disp(reshape(max(real(poles),[],1),2,[]));
assert(all(real(poles) < 0,'all'),'The full output-feedback hover loop is not Hurwitz.');
end

function sd = rate(s,fix,ref,C)
x = s(1:12); w = s(13:16); z = s(17:22); xh = s(23:31);
meas = x(7:12);
u = course_actuation(meas,x,w,ref,C);
[hd,feedback] = course_filter(xh,u(1),meas,x(1:3),fix,C);
[wd,zd] = course_d_law(meas,x,feedback,ref,z,C);
sd = [course_plant_rate(x,u,zeros(3,1),C); wd; zd; hd];
end
