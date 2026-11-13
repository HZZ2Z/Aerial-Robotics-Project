function eta = rot2euler(R)
%ROT2EULER ZYX Euler angles [phi;theta;psi] from body-to-inertial R.
eta = [atan2(R(3,2), R(3,3)); -asin(min(1,max(-1,R(3,1)))); atan2(R(2,1), R(1,1))];
end
