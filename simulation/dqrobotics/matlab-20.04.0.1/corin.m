clc;clear;
include_namespace_dq
arm_DH_theta = [0,pi/2, 0,pi/2, 0];
arm_DH_d = [0.147, 0, 0, 0, 0.218];
arm_DH_a = [0, 0.155, 0.135, 0, 0];
arm_DH_alpha = [pi/2, 0, 0,pi/2, 0];
arm_DH_matrix = [arm_DH_theta; arm_DH_d; arm_DH_a;arm_DH_alpha];
arm = DQ_SerialManipulator(arm_DH_matrix ,'standard'); 

q = [0, 0, 0, 0, 0,0,0,0 ,   0, 0, 0, 0, 0,0,0,0];
base = DQ_HolonomicBase();
base2 = DQ_HolonomicBase();

r =cos(pi/2) + j_*sin(pi/2);
p = 0.1*i_ + 2*j_ + 0.3*k_;
xd = r + E_*0.5*p*r;

% base.set_frame_displacement(xd);
 robot1 = DQ_WholeBody(base);
 robot1.add(arm);
 
 
 base2.set_frame_displacement(xd)
 robot2 = DQ_WholeBody(base2);
 robot2.add(arm);
 
wholerobot = DQ_CooperativeDualTaskSpace(robot1,robot2);
a = wholerobot.relative_pose(q)
a = wholerobot.absolute_pose(q)
a1 = wholerobot.pose1(q)
a2 = wholerobot.pose2(q)

% figure(1);
% plot(robot1,q);
% figure(2);
% plot(robot2,q);