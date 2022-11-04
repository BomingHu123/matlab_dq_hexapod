clc;clear;
include_namespace_dq

% q = [0, 0, 0, 0,    0, 0, 0, 0,     0, 0, 0, 0,     0, 0, 0, 0,     0, 0, 0, 0,     0, 0, 0, 0];
q = [0,0,0,0,0,0    0,0,0,0,0,0     0,0,0,0,0,0     0,0,0,0,0,0     0,0,0,0,0,0     0,0,0,0,0,0];
% leg = CorinLeg();
front2back = 0.115*2;
left2right = 0.09*2;

arm_DH_theta = [0,0,pi/2];
arm_DH_d = [0, 0, 0];
arm_DH_a = [0.077, 0.15, 0.17];
arm_DH_alpha = [pi/2, 0, 0];
arm_DH_matrix = [arm_DH_theta; arm_DH_d; arm_DH_a;arm_DH_alpha];
leg = DQ_SerialManipulator(arm_DH_matrix ,'standard'); 

base1 = DQ_HolonomicBase();
robot = DQ_WholeBody(base1);
robot.add(leg);
robot.fkm(q);
robot.pose_jacobian(q)
robot.plot(q);

