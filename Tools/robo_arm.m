% first robot

theta_1 = pi/2;
theta_2 = pi/2;
theta_3 = pi/2;
theta_4 = pi/2;
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
% robot.plot([pi/4 pi/4 -pi/4 -pi/4]);
% robot.teach();
[fir, sec] = robot.fkine([pi/4 pi/4 -pi/4 -pi/4]);
sec
robot.trchain('sym')