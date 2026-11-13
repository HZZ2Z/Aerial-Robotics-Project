function R = euler2rot(eta)
%EULER2ROT Body-to-inertial rotation from ZYX Euler angles eta=[phi;theta;psi].
cf = cos(eta(1)); sf = sin(eta(1));
ct = cos(eta(2)); st = sin(eta(2));
cp = cos(eta(3)); sp = sin(eta(3));
R = [cp*ct, cp*st*sf - sp*cf, cp*st*cf + sp*sf;
     sp*ct, sp*st*sf + cp*cf, sp*st*cf - cp*sf;
     -st,   ct*sf,            ct*cf];
end
