function REF = course_reference_table(P, dt, tfinal)
%COURSE_REFERENCE_TABLE 在每个 RK4 半步长 (dt/2) 处预采样参考轨迹，供 Simulink From Workspace 模块查表。
persistent cacheKey cacheRef
h = dt/2;
if strcmp(P.mis.mode, 'mission')
    % 课程主任务：直接调用 student_kit 中的 mission_reference 并缓存结果
    key = [h P.g tfinal];
    if ~isequal(cacheKey, key)
        r = mission_reference(h, P.g);
        assert(abs(r.t(end)-P.sim.tfinal) <= h, 'Mission duration and teacher reference differ.');
        cacheRef = [r.t, r.p, r.v, r.a, r.psi, r.psid, r.psidd, ...
                    r.att_ff, r.att_ff_d, r.att_ff_dd];
        cacheKey = key;
    end
    REF = cacheRef(cacheRef(:,1) <= tfinal+eps(tfinal), :);
    return
end

% 扩展实验模式（如 q_maneuver / hover_validation）：调用 gen_ref 并数值求导前馈倾角速率/加速度
REF = zeros(numel(t),19);
REF(:,1) = t;
for k = 1:numel(t)
    [p,v,a,yaw,dyaw,ddyaw] = gen_ref(t(k), P);
    c = cos(yaw); s = sin(yaw);
    acc = [c s; -s c]*a(1:2);
    REF(k,2:15) = [p;v;a;yaw;dyaw;ddyaw;acc(2)/P.g;-acc(1)/P.g]';
end
for k = 14:15
    REF(:,k+2) = gradient(REF(:,k), h);
    REF(:,k+4) = gradient(REF(:,k+2), h);
end
end
