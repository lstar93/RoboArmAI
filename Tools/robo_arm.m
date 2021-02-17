% Forward kinematics

theta_1 = pi/4;
theta_2 = pi/4;
theta_3 = pi/4;
theta_4 = pi/4;
% theta_5 = pi/2;

d_1 = 2;
d_2 = 0;
d_3 = 0;
d_4 = 0;
% d_5 = 0;

a_1 = 0;
a_2 = 2;
a_3 = 2;
a_4 = 2;
% a_5 = 0;

alpha_1 = pi/2;
alpha_2 = 0;
alpha_3 = 0;
alpha_4 = 0;
% alpha_5 = 0;

joint_1 = [theta_1 d_1 a_1 alpha_1];
joint_2 = [theta_2 d_2 a_2 alpha_2];
joint_3 = [theta_3 d_3 a_3 alpha_3];
joint_4 = [theta_4 d_4 a_4 alpha_4];
% joint_5 = [theta_5 d_5 a_5 alpha_5];

th1 = 0;
th2 = pi/2;
th3 = -pi/3;
th4 = -pi/3;

[th1; th2; th3; th4]

thrid_joint_kine = SerialLink([joint_1; joint_2; joint_3]);
% robot.plot([pi 0 pi/2]);
[thrd_joint, sec] = thrid_joint_kine.fkine([th1 th2 th3]);
% robot.teach();
% thrd_joint

e1 = 2;
e2 = 2;
e3 = 2;
e4 = 2;

robot = SerialLink([joint_1; joint_2; joint_3; joint_4]);
robot.plot([th1 th2 th3 th4]);
[fir2, sec2] = robot.fkine([th1 th2 th3 th4]);
% robot.teach();
% fir2
sec2

nofirst_joint = SerialLink([joint_1; joint_2; joint_3; joint_4]);
% robot2.plot([th1 th2 th3 th4]);
[fir3, sec3] = robot.fkine([0 th2 th3 th4]);
% robot.teach();
fir3.t;

% Inverse kinematics for 3rd joint
xc =  thrd_joint.t(1);
yc =  thrd_joint.t(2);
z  =  thrd_joint.t(3);

cos_j3 = (xc^2 + yc^2 + (z - e1)^2 - e2^2 - e3^2) / (2 * e2 * e3);
sin_j3 = sqrt(1 - cos_j3^2);

theta_3 = atan2(sin_j3, cos_j3);

theta_1 = atan2(yc, xc);

cos_j2 = ((xc/cos(theta_1)) * (-e2-(e3*cos_j3)) + ((z - e1)*e3*sin_j3)) / (-e2^2 - e3^2 - (2 * e2 * e3 * cos_j3));
sin_j2 = (sqrt(1-cos_j2^2));

theta_2 = atan2(sin_j2, cos_j2);

% theta_4 = theta_1 + theta_2 + theta_3;
% atan2(fir2.o(3), fir2.n(3))
% atan2(fir2.n(3), fir2.o(3))
% atan2(fir2.t(1), fir2.t(2))
% atan2(fir2.t(2), fir2.t(1))
% atan2(fir2.t(3), fir2.t(1))
% theta_4 = -(atan2(fir2.t(3), fir2.t(1)) + theta_3);
% [theta_1, theta_2, theta_3, theta_4];


fjoint = SerialLink([joint_1; joint_2]);
[fjointfk, tmp1] = fjoint.fkine([th1 th2]);
% fjoint.plot([th1 th2])
xb = fjointfk.t(1);
yb = fjointfk.t(2);
zb = fjointfk.t(3);
[xb, yb, zb];

xd = fir2.t(1);
yd = fir2.t(2);
zd = fir2.t(3);
[xd, yd, zd];

BD = sqrt((zd - zb)^2 + (yd - yb)^2 + (xd - xb)^2);

a1 = 2;
a2 = 2;
a3 = 2;
a4 = 2;

gamma = acos((a3^2 + a4^2 - BD^2) / (2 * a3 * a4));
theta_4 = pi - gamma;

[theta_1, theta_2, theta_3, theta_4];