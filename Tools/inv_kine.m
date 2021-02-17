% syms xc yc z e1 e2 e3
% cos_j3 = (xc^2 + yc^2 + (z - e1)^2 - e2^2 - e3^2) / (2 * e2 * e3);
% sin_j3 = sqrt(1 - cos_j3^2);
% theta_3 = atan2(sin_j3, cos_j3);
% theta_1 = atan2(yc, xc);
% cos_j2 = ((xc/cos(theta_1)) * (-e2-(e3*cos_j3)) + ((z - e1)*e3*sin_j3)) / (-e2^2 - e3^2 - (2 * e2 * e3 * cos_j3));
% sin_j2 = abs((sqrt(1-cos_j2^2)));
% theta_2 = atan2(sin_j2, cos_j2);
% [theta_1, cos_j2, theta_3]


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

robot = SerialLink([joint_1; joint_2; joint_3; joint_4]);
[rkine, sec] = robot.fkine([pi pi/2 pi/3 pi/4]);
rkine

joint_q = SerialLink([joint_1; joint_2; joint_3]);
[qfkine, sec] = joint_q.fkine([pi pi/2 pi/3]);

d1 = 2;  % d1 = a1
a1 = d1; % d1 = a1
a2 = 2;
a3 = 2;
a4 = 2;
x = rkine.t(1);
y = rkine.t(2);
z = rkine.t(3);
Xq = qfkine.t(1);
Zq = qfkine.t(3);
CQ = sqrt((Xq - a1)^2 + y^2 + (Zq - d1)^2);
CP = sqrt((x - a1)^2 + y^2 + (z - d1)^2);
alpa = acos((CQ^2 + a2^2 - a3^2) / (2 * CQ * a2));
beta = acos((CQ^2 + CP^2 - a4^2) / (2 * CQ * CP));
gamma = atan((z - d1) / ((x - a1)^2 + y^2));
theta_1_inv = atan(y/x);
theta_2_inv = alpa + beta + gamma;
theta_3_inv = pi - acos((a2^2 + a3^2 - CQ^2) / (2 * a2 * a3));
theta_4_inv = 0;
inv_kine_angles = [theta_1_inv theta_2_inv theta_3_inv theta_4_inv];
inv_kine_angles