clc;clear;
include_namespace_dq

% r =cos(0) + k_*sin(0);
% p = 0*i_ + 0*j_ + 0*k_;
% r = cos(0.0384/2) + 0.001778 * i_ * sin(0.0384/2) + 0.99565 * j_ * sin(0.0384/2) + 0.093116* k_*sin(0.0384/2);
r = cos(pi/20) + 1* j_*sin(pi/20);
p = -0.02*i_ + 0.001*j_ + 0.07*k_;
desired_feethold1_pose = r + E_*0.5*p*r;
q_vec_0 = vec8(desired_feethold1_pose);

% r =cos(0) + k_*sin(0);
% p = 0*i_ + 0*j_ + 0*k_;
% desired_base_pose = r + E_*0.5*p*r;
% qd_vec = vec8(desired_base_pose);


% qd = [qd_vec;   0;0;2;    zeros(3,1);    zeros(3,1);     zeros(3,1);    zeros(3,1);  0;0;0];

current_base_pose = DQ(1);
q_vec = vec8(current_base_pose);

corin = DQ_Hexapod.kinematics();
% hexapod = DQ_Hexapod.kinematics();

q = [0;0;0;0;0;0;0;0;   0;0;0;    0;0;0;    0;0;0;     0;0;0;    0;0;0; 0;0;0];
corin.set_reference_to_first_feethold(q);
% ref_to_feethold1 = corin.get_reference_to_first_feethold();
% q = [vec8(ref_to_feethold1);   0;0;0;    0;0;0;    0;0;0;     0;0;0;    0;0;0; 0;0;0];
q = [1;0;0;0;0;0;0;0;   0;-pi/6;2*pi/3;    0;-pi/6;2*pi/3;    0;-pi/6;2*pi/3;     0;-pi/6;2*pi/3;    0;-pi/6;2*pi/3; 0;-pi/6;2*pi/3];
q0 = q;
xd = corin.fkm(q0);
xd(1) = desired_feethold1_pose;
% q0 = [vec8(ref_to_feethold1);   0;0;0;    0;0;0;    0;0;0;     0;0;0;    0;0;0; 0;0;0];

% a = corin.fkm(q,0)
% a = corin.fkm(q,1)
% a = corin.fkm(q,2)
% corin.fkm(q,3)
% corin.fkm(q,4)
% corin.fkm(q,5)
% corin.fkm(q,6)

corin.set_reference_to_first_feethold(q);

% x_origin = corin.fkm(q);
% xd = x_origin;
% r =cos(0) + k_*sin(0);
% p = 0*i_ + 0*j_ + 0.1*k_;
% desired_base_pose = r + E_*0.5*p*r;
% xd(1) = desired_base_pose;

% b = load('matlab.mat');
% b = load('q2_constraint.mat');

% xd = corin.fkm(b.a);



% base_0 = translation(corin.fkm(qd,0))
% leg1 = translation(corin.fkm(qd,1))
% leg2 = translation(corin.fkm(qd,2))
% leg3 = translation(corin.fkm(qd,3))
% leg4 = translation(corin.fkm(qd,4))
% leg5 = translation(corin.fkm(qd,5))
% leg6 = translation(corin.fkm(qd,6))
% a = corin.pose_jacobian(qd);

% corin.plot(qd);

% corin.set_reference_to_first_feethold(q);

x_current = corin.fkm(q);
x_absolute_pose = x_current(1);



t = 0.001;% sampling time
T = 0;
Q = [];
Time =[];
gain = 10;% controller gain
% gain = eye(48);
% gain(1:8, 1:8) = 100*eye(8);

% a = corin.get_estimated_base_velocity(q,q,t);

solver = DQ_QuadprogSolver;
control = DQ_ClassicQPController_Hexapod(corin,solver);
% control = DQ_PseudoinverseController(corin);
control.set_control_objective(ControlObjective.HexapodTask);

% % -------------------------------------------------------------------------
% % %-----------------calculate constraint matrix and vector-----------------
% % -------------------------------------------------------------------------

% [~,base_velocity] = corin.get_estimated_base(q,q,t);
% % vec_base = DQ(0);
% 
% [Constraint_matrix,Constraint_Vector] = corin.get_constraint_matrix_and_vector(q,base_velocity);
% Constraint_Vector = -Constraint_Vector;
% % control.set_inequality_constraint(Constraint_matrix,Constraint_Vector);
% control.set_equality_constraint(Constraint_matrix,Constraint_Vector);

% % % -----------------------------------------------------------------------

control.set_gain(gain);
control.set_stability_threshold(1e-7);
Xd= [];
for i = 1:6
    x = vec8(xd(i));
    Xd = [Xd;x];
end


% % -------------------------------------------------------------------------
% % %---------------------------Controller Part------------------------------
% % -------------------------------------------------------------------------
x_current_abs_pose1 =0;
C = [];
Q=[];
Q = [Q,q0];
Time = [];
Time = [Time,0];
while ~control.system_reached_stable_region()
    [u,c] = control.compute_setpoint_control_signal(q,Xd);
    last_q = q;
    q(9:26) = q(9:26) + t*u;
    C = [C,c];
    %     q = q + t*u;
    
    [x_current_abs_pose,base_velocity] = corin.get_estimated_base(q,last_q,t);
    
    q(1:8) = vec8(x_current_abs_pose);
    
    x_current_abs_pose1 = x_current_abs_pose;
    
%     [Constraint_matrix,Constraint_Vector] = corin.get_constraint_matrix_and_vector(q,base_velocity);
%     Constraint_Vector = -Constraint_Vector;
%     control.set_equality_constraint(Constraint_matrix,Constraint_Vector);
%     control.set_inequality_constraint(Constraint_matrix,Constraint_Vector);
    
    Q=[Q,q];
%         corin.set_reference_to_first_feethold(q);
    T = T + t;
    Time = [Time T];
    %     corin.plot(q);
end

corin.fkm(q,4)



% % -------------------------------------------------------------------------
% 
% resultqd_1 = corin.fkm(qd);
% resultq_1 =corin.fkm(q);
% Res_Xd = [];
% Res_X_Current = [];
% for i = 1:6
%     res_xd = translation(resultqd_1(i));
%     Res_Xd = [Res_Xd,res_xd];
%     res_x_current = translation(resultq_1(i));
%     Res_X_Current = [Res_X_Current,res_x_current];
% end



% resultq0_1 =corin.fkm(q0)
%
%
% corin.set_reference_to_first_feethold(qd);
% resultqd_2 = corin.fkm(qd)
% resultq_2 = corin.fkm(q)
% resultq0_2 = corin.fkm(q0)




% hold on;
% for i = 9:26
% plot(Time,Q(i,:))
% end


%--------------------------------------------------------------------------
%------------------------- plot the Xd and X_current-----------------------
%--------------------------------------------------------------------------
% Desired_Feet_to_Feet = zeros(6,length(Q));
% for i = 1:6
% vec_desired_feet_to_feet = vec3(translation(xd(i)));
% desired_feet1_to_feet2 = norm(vec_desired_feet_to_feet);
% Desired_Feet_to_Feet(i,1) = desired_feet1_to_feet2;
% end
%
%
% Feet1_to_Feet2 = zeros(6,length(Q));
% X_Current = [];
%
%
%
% for i = 1:length(Q)
%     x_current = corin.fkm(Q(:,i));
%     X_Current = [X_Current; x_current];
%     for j =1:6
%         vec_feet1_to_feet2 = vec3(translation(X_Current(i,j)));
%         feet1_to_feet2 = norm(vec_feet1_to_feet2);
%
%         Feet1_to_Feet2(j,i) = feet1_to_feet2;
%         Desired_Feet_to_Feet(j,i) = Desired_Feet_to_Feet(j,1);
%     end
% end
%
% for i = 1:6
%
% figure(i);
% hold on;
% plot(Time,Feet1_to_Feet2(i,:));
% plot(Time,Desired_Feet_to_Feet(i,:));
% %
%
% end


