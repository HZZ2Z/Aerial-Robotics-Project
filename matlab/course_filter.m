function [xd, feedback] = course_filter(x, T, meas, y, fix, P)
%COURSE_FILTER Steady-state continuous observer with a complete-outage mode.
if fix > 0.5
    L = P.L_rtk;
elseif fix >= 0
    L = P.L_deg;
else
    L = zeros(9,3);
end
R = euler_R(meas(1:3));
ua = -(T/P.m)*R(:,3) + [0; 0; P.g];
xd = P.A_kf*x + P.B_kf*ua + L*(y - P.C_kf*x);
feedback = [x(1:6); xd(1:3)];
end
