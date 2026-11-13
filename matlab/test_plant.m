%TEST_PLANT Open-loop verification of the Simulink quadrotor plant.
P = params();
mdl = build_plant(P);
out = sim(mdl);
p = out.plant_p; v = out.plant_v; eta = out.plant_eta;
fprintf('hover t=5s: p=[%.4f %.4f %.4f] v=[%.1e %.1e %.1e] eta=[%.1e %.1e %.1e]\n', ...
    p.Data(end,1), v.Data(end,1), eta.Data(end,1));
ok1 = max(abs(v.Data(end,:))) < 1e-6 && max(abs(eta.Data(end,:))) < 1e-6;
close_system(mdl, 0);
R0 = eye(3);
x0 = [zeros(3,1); zeros(3,1); R0(:); zeros(3,1)];
% open-loop torque response via ode45 (angles wrap: compare mod 2pi)
phi_an = 0.5*(0.2/P.J(1,1))*1;
[~, xx] = ode45(@(t,x) quad_dynamics(x, [P.Ttrim; 0.2; 0; 0], [0;0;0], P), [0 1], x0);
phi1 = rot2euler(reshape(xx(end,7:15),3,3));
fprintf('roll after 1 s of nx=0.2 (ode45): %.4f rad (analytic %.4f)\n', phi1(1), phi_an);
ok2 = abs(mod(phi1(1) - phi_an + pi, 2*pi) - pi) < 0.01;
% wind drag check: no feedback -> v converges to vw = 2 m/s, pitch stays 0
[~, x2] = ode45(@(t,x) quad_dynamics(x, [P.Ttrim; 0; 0; 0], [2;0;0], P), [0 8], x0);
th = rot2euler(reshape(x2(end,7:15),3,3));
vexp = 2*(1 - exp(-P.kd*8/P.m));
fprintf('wind 2 m/s: v_x(8s) = %.4f m/s (exp %.4f), pitch %.3f deg\n', x2(end,4), vexp, th(2)*180/pi);
ok3 = abs(x2(end,4) - vexp) < 0.05 && abs(th(2)) < 1e-3;
assert(ok1 && ok2 && ok3, 'plant test FAILED');
disp('PLANT TESTS PASSED');
