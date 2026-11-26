function ref = course_feedforward(ref, d, P)
%COURSE_FEEDFORWARD Optional disturbance compensation using smoothed estimates.
if P.ff_wind
    % Rotate d and both derivatives into the reference-heading frame.
    c = cos(ref(10)); s = sin(ref(10));
    Rt = [c, s; -s, c];
    G = [0, -1; 1, 0];
    dh = Rt*d(1:2);
    dhd = Rt*d(4:5) - ref(11)*G*dh;
    dhdd = Rt*d(7:8) - 2*ref(11)*G*dhd - ref(12)*G*dh + ref(11)^2*dh;
    correction = [0, -1; 1, 0]/P.g;
    ref(9) = ref(9) - d(3);
    ref(13:14) = ref(13:14) + correction*dh;
    ref(15:16) = ref(15:16) + correction*dhd;
    ref(17:18) = ref(17:18) + correction*dhdd;
end
end
