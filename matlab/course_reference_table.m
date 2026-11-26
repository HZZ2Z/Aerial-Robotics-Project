function REF = course_reference_table(P, dt, tfinal)
%COURSE_REFERENCE_TABLE Sample the teacher's reference at every RK4 stage.
persistent cacheKey cacheRef
h = dt/2;
if strcmp(P.mis.mode, 'mission')
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

t = (0:h:tfinal)';
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
