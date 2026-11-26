function [pref, vref, aref, psi, psid, psidd] = gen_ref(t, P)
%GEN_REF 生成至少 C^2 光滑的分段解析参考轨迹（用于扩展对比实验）。
%   所有轨迹段之间均采用五次多项式 smoothstep q5 拼接，保证位置、速度、加速度与航向角二阶连续：
%   1) 起飞爬升 -> 2) 悬停 -> 3) 平滑切入8字双纽线 -> 4) 两圈8字 -> 5) 平滑切出 -> 6) 悬停 -> 7) 降落
%   航向角 psi* 在 8 字飞行期间保持机头 x_B 切向于轨迹: atan2(ydot*, xdot*)。

% 模式 A：专用滚转-俯仰-偏航联合机动（用于对比精确 Q 与近似 Q=I）
% climb 3 s, hover, simultaneous diagonal position step (0,0)->(2,2) and
% 90 deg heading change over [5,7], hold, return over [9,11], hover, land.
if strcmp(P.mis.mode, 'q_maneuver')
    if t <= 3
        [q,qd,qdd] = q5(t/3);
        pref = [0;0;-2*q]; vref = [0;0;-2*qd/3]; aref = [0;0;-2*qdd/9];
        psi = 0; psid = 0; psidd = 0;
    elseif t <= 5
        pref = [0;0;-2]; vref = [0;0;0]; aref = [0;0;0]; psi = 0; psid = 0; psidd = 0;
    elseif t <= 7
        [q,qd,qdd] = q5((t-5)/2);
        pref = [2*q; 2*q; -2]; vref = [qd; qd; 0]; aref = [qdd/2; qdd/2; 0];
        psi = (pi/2)*q; psid = (pi/4)*qd; psidd = (pi/8)*qdd;
    elseif t <= 9
        pref = [2;2;-2]; vref = [0;0;0]; aref = [0;0;0];
        psi = pi/2; psid = 0; psidd = 0;
    elseif t <= 11
        [q,qd,qdd] = q5((t-9)/2);
        pref = [2*(1-q); 2*(1-q); -2]; vref = [-qd; -qd; 0]; aref = [-qdd/2; -qdd/2; 0];
        psi = (pi/2)*(1-q); psid = -(pi/4)*qd; psidd = -(pi/8)*qdd;
    elseif t <= 12
        pref = [0;0;-2]; vref = [0;0;0]; aref = [0;0;0]; psi = 0; psid = 0; psidd = 0;
    else
        [q,qd,qdd] = q5((t-12)/2);
        pref = [0;0;-2*(1-q)]; vref = [0;0;2*qd/2]; aref = [0;0;2*qdd/4];
        psi = 0; psid = 0; psidd = 0;
    end
    return
end

% static hover validation (integral anti-wind test): 5 s climb to 2 m,
% then hold position until P.mis.t_hv; mean wind steps at P.wind.t0.
if strcmp(P.mis.mode, 'hover_validation')
    tup = P.mis.tup;  h = P.mis.h;
    if t <= tup
        [q, qd, qdd] = q5(t/tup);
        pref = [0;0;-h*q]; vref = [0;0;-h*qd/tup]; aref = [0;0;-h*qdd/tup^2];
    else
        pref = [0;0;-h]; vref = [0;0;0]; aref = [0;0;0];
    end
    psi = 0; psid = 0; psidd = 0;
    return
end

h = P.mis.h; tup = P.mis.tup; thov = P.mis.thov; tb = P.mis.tb;
a = P.mis.a; Om = 2*pi/P.mis.T8; tl = P.mis.laps*P.mis.T8; tland = P.mis.tland;

t1 = tup + thov;                 % blend-in start
t2 = t1 + tb;                    % lemniscate start
t3 = t2 + tl;                    % lemniscate end / blend-out start
t4 = t3 + tb;                    % hover 2
t5 = t4 + thov;                  % landing start

% --- piecewise position/velocity/acceleration ---
if t <= tup                                    % take-off climb
    [q,qd,qdd] = q5(t/tup);
    pref = [0;0;-h*q]; vref = [0;0;-h*qd/tup]; aref = [0;0;-h*qdd/tup^2];
    psiA = 0; psiAd = 0; psiAdd = 0; psB = 0; psBd = 0; psBdd = 0; w = 0; wd = 0; wdd = 0;
elseif t <= t1                                 % hover 1
    pref = [0;0;-h]; vref = [0;0;0]; aref = [0;0;0];
    psiA = 0; psiAd = 0; psiAdd = 0; psB = 0; psBd = 0; psBdd = 0; w = 0; wd = 0; wdd = 0;
elseif t <= t2                                 % blend hover -> eight
    [q,qd,qdd] = q5((t-t1)/tb);  w = q; wd = qd/tb; wdd = qdd/tb^2;
    [pB,vB,aB,psB,psBd,psBdd] = lem(t - t2, a, Om, -h);
    A = [0;0;-h]; vA = [0;0;0]; aA = [0;0;0];
    pref = A + q*(pB-A);  vref = vA + q*(vB-vA) + wd*(pB-A);
    aref = aA + q*(aB-aA) + 2*wd*(vB-vA) + wdd*(pB-A);
    psiA = 0; psiAd = 0; psiAdd = 0;
elseif t <= t3                                 % figure-eight, 2 laps
    [pref,vref,aref,psi,psid,psidd] = lem(t - t2, a, Om, -h);
    return
elseif t <= t4                                 % blend eight -> hover
    [q,qd,qdd] = q5((t-t3)/tb);  w = q; wd = qd/tb; wdd = qdd/tb^2;
    [pA,vA,aA,psA,psAd,psAdd] = lem(t - t2, a, Om, -h);
    B = [0;0;-h]; vB = [0;0;0]; aB = [0;0;0];
    pref = pA + q*(B-pA);  vref = vA + q*(vB-vA) + wd*(B-pA);
    aref = aA + q*(aB-aA) + 2*wd*(vB-vA) + wdd*(B-pA);
    psiA = psA; psiAd = psAd; psiAdd = psAdd; psB = 0; psBd = 0; psBdd = 0;
elseif t <= t5                                 % hover 2
    pref = [0;0;-h]; vref = [0;0;0]; aref = [0;0;0];
    psiA = 0; psiAd = 0; psiAdd = 0; psB = 0; psBd = 0; psBdd = 0; w = 0; wd = 0; wdd = 0;
else                                           % landing
    [q,qd,qdd] = q5((t-t5)/tland);
    pref = [0;0;-h*(1-q)]; vref = [0;0;h*qd/tland]; aref = [0;0;h*qdd/tland^2];
    psiA = 0; psiAd = 0; psiAdd = 0; psB = 0; psBd = 0; psBdd = 0; w = 0; wd = 0; wdd = 0;
    psi = 0; psid = 0; psidd = 0; return
end

% --- blended heading: psi = (1-w)*psiA + w*psB (angle wrapped) ---
dpsi = wrapPi(psB - psiA);
psi  = psiA + w*dpsi;
psid = psiAd + wd*dpsi + w*(psBd - psiAd);
psidd = psiAdd + wdd*dpsi + 2*wd*(psBd - psiAd) + w*(psBdd - psiAdd);
end

% ---- figure-eight (lemniscate of Gerono) at constant altitude z0, local time tau ----
function [p,v,a,psi,psid,psidd] = lem(tau, a, Om, z0)
x  = a*sin(Om*tau);        y  = a*sin(Om*tau)*cos(Om*tau);
xd = a*Om*cos(Om*tau);     yd = a*Om*cos(2*Om*tau);
xdd = -a*Om^2*sin(Om*tau); ydd = -2*a*Om^2*sin(2*Om*tau);
xddd = -a*Om^3*cos(Om*tau); yddd = -4*a*Om^3*cos(2*Om*tau);
p = [x;y;z0]; v = [xd;yd;0]; a = [xdd;ydd;0];
psi = atan2(yd, xd);
D = xd^2 + yd^2;
N = xd*ydd - yd*xdd;
psid = N/D;
Ndd = xd*yddd - yd*xddd;               % d/dt(xd*ydd - yd*xdd) = xd*yddd - yd*xddd
psidd = (Ndd*D - N*2*(xd*xdd + yd*ydd)) / D^2;
end

% ---- quintic smoothstep and derivatives, s in [0,1] ----
function [q,qd,qdd] = q5(s)
s = min(max(s,0),1);
q   = 6*s^5 - 15*s^4 + 10*s^3;
qd  = 30*s^4 - 60*s^3 + 30*s^2;
qdd = 120*s^3 - 180*s^2 + 60*s;
end

function d = wrapPi(a)
d = mod(a + pi, 2*pi) - pi;
end
