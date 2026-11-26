function J = course_jacobian(f, x)
%COURSE_JACOBIAN Central difference for verification around an equilibrium.
h = 1e-6;
J = zeros(numel(f(x)),numel(x));
for k = 1:numel(x)
    dx = zeros(size(x));
    dx(k) = h;
    J(:,k) = (f(x+dx)-f(x-dx))/(2*h);
end
end
