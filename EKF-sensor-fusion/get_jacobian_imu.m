clear all
syms x y z theta1 theta2 theta3 dt constant

ax_bar = [x/dt - constant]/dt;
ay_bar = [y/dt - constant]/dt;
az_bar = [z/dt - constant]/dt;

c1 = cos(theta1);
c2 = cos(theta2);
c3 = cos(theta3);
s1 = sin(theta1);
s2 = sin(theta2);
s3 = sin(theta3);

r11 = c2*c3;
r12 = c3*s2*s1-s3*c1;
r13 = c3*s2*c1+s3*s1;
r21 = s3*c2;
r22 = s3*s2*s1+c3*c1;
r23 = s3*s2*c1-c3*s1;
r31 = -s2;
r32 = c2*s1;
r33 = c3*c1;

rot = [r11 r12 r13; r21 r22 r23; r31 r32 r33];

a_bar = [ax_bar; ay_bar; az_bar];

a_hat = rot*a_bar;

h = [a_hat; theta1; theta2; theta3];
X = [x; y; z; theta1; theta2; theta3];

H = jacobian(h,X);