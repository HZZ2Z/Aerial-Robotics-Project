function [A, B] = lin_hover(P)
%LIN_HOVER Hover design in heading coordinates, with position and yaw integrals.
I = eye(3); Z = zeros(3);
G = P.g*[0 -1 0; 1 0 0; 0 0 0];
Ap = [Z I Z Z; Z -P.kd/P.m*I G Z; Z Z Z I; Z Z Z Z];
Bp = [zeros(5,4); -1/P.m 0 0 0; zeros(3,4); zeros(3,1) P.J\I];
E = eye(12);
E = E([1:3 9],:);
A = [zeros(4), E; zeros(12,4), Ap];
B = [zeros(4); Bp];

plant = struct('m_true',P.m,'J_true',P.J,'g',P.g,'kd',P.kd);
J = course_jacobian(@rate, zeros(20,1));
assert(norm(J-[A B],inf) < 1e-6, 'Hover blocks differ from the nonlinear Jacobian.');
r = rank(ctrb(A,B));
assert(r == 16, 'The augmented hover plant must be controllable.');
fprintf('Hover Jacobian residual %.2g, controllability rank %d/16\n',norm(J-[A B],inf),r);

    function dx = rate(z)
        x = z(5:16);
        u = z(17:20) + [P.Ttrim; 0; 0; 0];
        dx = [E*x; course_plant_rate(x,u,zeros(3,1),plant)];
    end
end
