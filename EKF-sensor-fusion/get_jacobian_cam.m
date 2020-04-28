clear all
syms x y z q0 q1 q2 q3 real;


h = [x; y; z; 
        atan2(2*(q3*q0+q1*q2), 1-2*(q0^2+q1^2)); 
        asin(2*(q3*q1-q2*q0));
        atan(2*(q3*q2+q0*q1), 1-2*(q1^2+q2^2))];
X = [x; y; z; q0; q1; q2; q3];

H = jacobian(h,X)