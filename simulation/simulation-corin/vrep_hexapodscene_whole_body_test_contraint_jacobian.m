% Example used in the paper Adorno and Marinho. 2020. â€œDQ Robotics: A
% Library for Robot Modeling and Control.â€? IEEE Robotics & Automation
% Magazine, https://doi.org/10.1109/MRA.2020.2997920.
%
% Usage: Assuming that V-REP is already working with Matlab,
%        1) open the file vrep_scene_felt_pen_official_scene.ttt on V-REP;
%        2) run this file;
%        3) adjust the parameters below if you want to change the
%        trajectory parameters, simulation time, etc.

% (C) Copyright 2020 DQ Robotics Developers
%
% This file is part of DQ Robotics.
%
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
%
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
%
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ieee.org

function [Foothold1_to_foothold2_vrep_trans, Foothold2_to_foothold3_vrep_trans, Foothold3_to_foothold4_vrep_trans, Foothold4_to_foothold5_vrep_trans, Foothold5_to_foothold6_vrep_trans,...
    Foothold1_to_foothold2_matlab_trans,Foothold2_to_foothold3_matlab_trans,Foothold3_to_foothold4_matlab_trans,Foothold4_to_foothold5_matlab_trans,...
    Foothold5_to_foothold6_matlab_trans,Body_copp_vrep_trans,Body_matlab_trans,Norm_of_error]= vrep_hexapodscene_whole_body_test_contraint_jacobian(simulation_parameters)

include_namespace_dq;

%% Simulation convenience flags
SHOW_FRAMES = simulation_parameters.show_frames;

%% Initialize V-REP interface
vi = DQ_VrepInterface;
vi.disconnect_all(); % For testing, can be removed for release
vi.connect('127.0.0.1',19997);
vi.synchronous();
vi.start_simulation();

%% Initialize VREP Robots
corin_hexapod = DQVrepHexapod('Hexapod',vi);
% corin_hexapod = DQVrepCorinHexapod('Corin',vi);
% youbot_vreprobot = YouBotVrepRobot('youBot',vi);

%% Load DQ Robotics kinematics
corin  = corin_hexapod.kinematics();
% youbot = youbot_vreprobot.kinematics();

%% Initialize controllers
solver = DQ_QuadprogSolver;
corin_controller = DQ_ClassicQPController_Hexapod(corin,solver);
corin_controller.set_control_objective(ControlObjective.HexapodTask);
corin_controller.set_gain(0.01);
corin_controller.set_damping(0.01);

sampling_time = 0.05; % V-REP's sampling time is 50 ms.
total_time = simulation_parameters.total_time; % Total time, in seconds.

%% Set the initial robots configurations
% lwr4_q  = [0; 1.7453e-01; 0; 1.5708e+00; 0; 2.6273e-01; 0];


%% constant
include_namespace_dq

max_iterations = round(total_time/sampling_time);

corin_tracking_error_norm = zeros(1,max_iterations);
corin_control_inputs = zeros(corin.get_dim_configuration_space(),...
    max_iterations);
corin_q_vector = zeros(26, ...
    max_iterations);

Foothold1_to_foothold2_vrep_trans = zeros(3,max_iterations);
Foothold2_to_foothold3_vrep_trans = zeros(3,max_iterations);
Foothold3_to_foothold4_vrep_trans = zeros(3,max_iterations);
Foothold4_to_foothold5_vrep_trans = zeros(3,max_iterations);
Foothold5_to_foothold6_vrep_trans = zeros(3,max_iterations);
Body_copp_vrep_trans = zeros(3,max_iterations);

Foothold1_to_foothold2_vrep_rotation = zeros(4,max_iterations);
Foothold2_to_foothold3_vrep_rotation = zeros(4,max_iterations);
Foothold3_to_foothold4_vrep_rotation = zeros(4,max_iterations);
Foothold4_to_foothold5_vrep_rotation = zeros(4,max_iterations);
Foothold5_to_foothold6_vrep_rotation = zeros(4,max_iterations);
Body_vrep_rotation = zeros(4,max_iterations);

Foothold1_to_foothold2_matlab_trans = zeros(3,max_iterations);
Foothold2_to_foothold3_matlab_trans = zeros(3,max_iterations);
Foothold3_to_foothold4_matlab_trans = zeros(3,max_iterations);
Foothold4_to_foothold5_matlab_trans = zeros(3,max_iterations);
Foothold5_to_foothold6_matlab_trans = zeros(3,max_iterations);
Body_matlab_trans = zeros(3,max_iterations);

Foothold1_to_foothold2_matlab_rotation = zeros(4,max_iterations);
Foothold2_to_foothold3_matlab_rotation = zeros(4,max_iterations);
Foothold3_to_foothold4_matlab_rotation = zeros(4,max_iterations);
Foothold4_to_foothold5_matlab_rotation = zeros(4,max_iterations);
Foothold5_to_foothold6_matlab_rotation = zeros(4,max_iterations);
Body_matlab_rotation = zeros(4,max_iterations);

Vec_x_ref2base_dot_matlab = zeros(8,max_iterations);
Vec_x_ref2base_dot = zeros(8,max_iterations);

Vec_x_ref2foothold_dot_matlab = zeros(8,max_iterations);
Vec_x_ref2foothold_dot = zeros(8,max_iterations);

Vec_norm_error = zeros(1,max_iterations);

Angular_velocity_foothold1 = zeros(3,max_iterations);
Linear_velocity_foothold1 = zeros(3,max_iterations);
Angular_velocity_foothold1_Vrep = zeros(3,max_iterations);
Linear_velocity_foothold1_Vrep = zeros(3,max_iterations);
Angular_velocity_body_Vrep = zeros(3,max_iterations);
Linear_velocity_body_Vrep = zeros(3,max_iterations);
linear_velocity_foothold1_right = zeros(3,max_iterations);
linear_velocity_foothold2_right = zeros(3,max_iterations);
linear_velocity_foothold3_right = zeros(3,max_iterations);
linear_velocity_foothold4_right = zeros(3,max_iterations);
linear_velocity_foothold5_right = zeros(3,max_iterations);
linear_velocity_foothold6_right = zeros(3,max_iterations);
linear_velocity_foothold1_jacobian = zeros(3,max_iterations);
linear_velocity_foothold2_jacobian = zeros(3,max_iterations);
linear_velocity_foothold3_jacobian = zeros(3,max_iterations);
linear_velocity_foothold4_jacobian = zeros(3,max_iterations);
linear_velocity_foothold5_jacobian = zeros(3,max_iterations);
linear_velocity_foothold6_jacobian = zeros(3,max_iterations);


current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
last_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
current_corin_base = vi.get_object_pose('/hexapod/body',-1,vi.OP_BLOCKING);
last_corin_base = vi.get_object_pose('/hexapod/body',-1,vi.OP_BLOCKING);
Norm_of_error = [];

corin_q = corin_hexapod.get_q_from_vrep();
current_leg1_q = corin_q(9:11);
last_leg1_q = corin_q(9:11);
last_leg_q = corin_q(9:26)
ii = 1;

    last_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
    last_foothold2 = vi.get_object_pose('/hexapod/footTip1',-1,vi.OP_BLOCKING);
    last_foothold3 = vi.get_object_pose('/hexapod/footTip2',-1,vi.OP_BLOCKING);
    last_foothold4 = vi.get_object_pose('/hexapod/footTip3',-1,vi.OP_BLOCKING);
    last_foothold5 = vi.get_object_pose('/hexapod/footTip4',-1,vi.OP_BLOCKING);
    last_foothold6 = vi.get_object_pose('/hexapod/footTip5',-1,vi.OP_BLOCKING);
%     foothold1 = corin_hexapod.get_reference_feethold1_from_vrep;
% %     corin.set_reference_to_first_feethold(corin_q);
% 
%     corin.set_reference_to_first_feethold_vrep(foothold1);
    a = 1;
%% Control loop
for t=0:sampling_time:total_time
    %% get information from CoppeliaSim
    vi.synchronous_trigger();
    pause(0.05);
    
    %% get information from Matlab
    foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
%     foothold1 = corin_hexapod.get_reference_feethold1_from_vrep;
%     corin.set_reference_to_first_feethold(corin_q);
    corin.set_reference_to_first_feethold_vrep(foothold1);
    
    corin_q = corin_hexapod.get_q_from_vrep();
    current_leg1_q = corin_q(9:11);
    current_leg_q = corin_q(9:26);
%     if corin_q(8+1) <= 0.1
%         corin_q(8+1) = corin_q(8+1) + 0.002;
%     end
%     if corin_q(8+2) <= 0
%         corin_q(8+2) = corin_q(8+2) + 0.002;
%     end
%     if corin_q(8+3) >= 0
%         corin_q(8+3) = corin_q(8+3) - 0.005;
%     end
    fkm_res = corin.fkm(corin_q);
    fkm1 = fkm_res(1);
    fkm2 = fkm_res(2);
    fkm3 = fkm_res(3);
    fkm4 = fkm_res(4);
    fkm5 = fkm_res(5);
    fkm6 = fkm_res(6);
    
    trans_fkm1 = fkm1.translation;
    trans_fkm2 = fkm2.translation;
    trans_fkm3 = fkm3.translation;
    trans_fkm4 = fkm4.translation;
    trans_fkm5 = fkm5.translation;
    trans_fkm6 = fkm6.translation;
    
    rotation_fkm1 = fkm1.rotation;
    rotation_fkm2 = fkm2.rotation;
    rotation_fkm3 = fkm3.rotation;
    rotation_fkm4 = fkm4.rotation;
    rotation_fkm5 = fkm5.rotation;
    rotation_fkm6 = fkm6.rotation;
    
    vec_trans_fkm1 = vec8(trans_fkm1);
    vec_trans_fkm2 = vec8(trans_fkm2);
    vec_trans_fkm3 = vec8(trans_fkm3);
    vec_trans_fkm4 = vec8(trans_fkm4);
    vec_trans_fkm5 = vec8(trans_fkm5);
    vec_trans_fkm6 = vec8(trans_fkm6);
    
    vec_rotation_fkm1 = vec8(rotation_fkm1);
    vec_rotation_fkm2 = vec8(rotation_fkm2);
    vec_rotation_fkm3 = vec8(rotation_fkm3);
    vec_rotation_fkm4 = vec8(rotation_fkm4);
    vec_rotation_fkm5 = vec8(rotation_fkm5);
    vec_rotation_fkm6 = vec8(rotation_fkm6);
    
    Foothold1_to_foothold2_matlab_trans(:,ii) = vec_trans_fkm2(2:4);
    Foothold2_to_foothold3_matlab_trans(:,ii) = vec_trans_fkm3(2:4);
    Foothold3_to_foothold4_matlab_trans(:,ii) = vec_trans_fkm4(2:4);
    Foothold4_to_foothold5_matlab_trans(:,ii) = vec_trans_fkm5(2:4);
    Foothold5_to_foothold6_matlab_trans(:,ii) = vec_trans_fkm6(2:4);
    Body_matlab_trans(:,ii) = vec_trans_fkm1(2:4);
    
    Foothold1_to_foothold2_matlab_rotation(:,ii) = vec_rotation_fkm2(1:4);
    Foothold2_to_foothold3_matlab_rotation(:,ii) = vec_rotation_fkm3(1:4);
    Foothold3_to_foothold4_matlab_rotation(:,ii) = vec_rotation_fkm4(1:4);
    Foothold4_to_foothold5_matlab_rotation(:,ii) = vec_rotation_fkm5(1:4);
    Foothold5_to_foothold6_matlab_rotation(:,ii) = vec_rotation_fkm6(1:4);
    Body_matlab_rotation(:,ii) = vec_rotation_fkm1(1:4);
    
    a = a+1;
    
    
    %%
    current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
    current_corin_base = vi.get_object_pose('/hexapod/body',-1,vi.OP_BLOCKING);
    [x_ref2f1_dot,angular_velocity_foothold1,linear_velocity_foothold1] = compute_estimated_velocity(current_corin_foothold1,last_corin_foothold1,sampling_time);
    [x_ref2base_dot_right,~,~] = compute_estimated_velocity(current_corin_base,last_corin_base,sampling_time);
    [linear_velocity_foothold1_vrep,angular_velocity_foothold1_vrep] = vi.get_body_velocity_from_vrep('/hexapod/footTip0');
        [linear_velocity_body_vrep,angular_velocity_body_vrep] = vi.get_body_velocity_from_vrep('/hexapod/body');
    
        
    vrep_base_trans = current_corin_base.translation;
    vrep_base_rotation = current_corin_base.rotation;
    vec_vrep_base_trans = vec8(vrep_base_trans);
    vec_vrep_base_rotation = vec8(vrep_base_rotation);
    Body_copp_vrep_trans(:,ii) = vec_vrep_base_trans(2:4);
    Body_vrep_rotation(:,ii) = vec_vrep_base_rotation(1:4);    
    
    
    Angular_velocity_foothold1(:,ii) = angular_velocity_foothold1;
    Linear_velocity_foothold1(:,ii) = linear_velocity_foothold1;
    Angular_velocity_foothold1_Vrep(:,ii) = angular_velocity_foothold1_vrep;
    Linear_velocity_foothold1_Vrep(:,ii) = linear_velocity_foothold1_vrep;
    Angular_velocity_body_Vrep(:,ii) = angular_velocity_body_vrep;
Linear_velocity_body_Vrep(:,ii) = linear_velocity_body_vrep;
    
    matlab_jacobian = corin.pose_jacobian(corin_q);
    matlab_jacobian_2 = corin.pose_jacobian_test(corin_q,current_corin_base);

    jacobian_abs = matlab_jacobian(1:8,1:11);
    
    vec_x_ref2f1_dot = vec8(x_ref2f1_dot);
    vec_abs = zeros(11,1);
    vec_abs(1:8) = vec_x_ref2f1_dot;
    leg1_q_dot = (current_leg1_q - last_leg1_q)/sampling_time;
    vec_abs(9:11) = leg1_q_dot;
    
    vec_x_ref2base_dot_matlab = jacobian_abs * vec_abs; 
    vec_x_ref2base_dot_right = vec8(x_ref2base_dot_right);
    error = vec_x_ref2base_dot_matlab - vec_x_ref2base_dot_right;
    
    vec_x_ref2foothold1_dot_matlab = matlab_jacobian_2 * leg1_q_dot;
    
    Vec_x_ref2foothold_dot_matlab(:,ii) = vec_x_ref2foothold1_dot_matlab;
    Vec_x_ref2foothold_dot(:,ii) = vec_x_ref2f1_dot;
    
    
    Vec_norm_error(:,ii) = norm(error);
    Vec_x_ref2base_dot_matlab(:,ii) = vec_x_ref2base_dot_matlab;
    Vec_x_ref2base_dot(:,ii) = vec_x_ref2base_dot_right;
%     
%     DQ_vec_x_ref2base_dot_matlab = DQ(vec_x_ref2base_dot_matlab);
%     estimated_velocity =0.5 * xi * current_pose;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%2023/04/20 for constraint jacobian test%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    current_base = vi.get_object_pose('/hexapod/body/',-1,vi.OP_BLOCKING);
    fkm_res = corin.fkm(corin_q);
    [x_ref2base_dot_approximation,angular_w_matlab,linear_v_matlab] = compute_estimated_velocity(current_corin_base,last_corin_base,sampling_time);
    [Constraint_matrix,Constraint_Vector,J_whole] = corin.get_constraint_matrix_and_vector(corin_q,x_ref2base_dot_approximation);
%      corin_controller.set_inequality_constraint(Constraint_matrix,Constraint_Vector);
%     corin_controller.set_equality_constraint(Constraint_matrix,Constraint_Vector);
    vec_contraint = zeros(26,1);
    vec_contraint(1:8) = vec_x_ref2base_dot_right;
    leg_q_dot = (current_leg_q - last_leg_q)/sampling_time;
    vec_contraint(9:26) = leg_q_dot;
    foothold_trans_dot = J_whole * vec_contraint;
    linear_velocity_foothold1_jacobian(:,ii) = foothold_trans_dot(1:3);
   linear_velocity_foothold2_jacobian(:,ii) = foothold_trans_dot(4:6);
   linear_velocity_foothold3_jacobian(:,ii) = foothold_trans_dot(7:9);
   linear_velocity_foothold4_jacobian(:,ii) = foothold_trans_dot(10:12);
   linear_velocity_foothold5_jacobian(:,ii) = foothold_trans_dot(13:15);
   linear_velocity_foothold6_jacobian(:,ii) = foothold_trans_dot(16:18);
      
    foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
    foothold2 = vi.get_object_pose('/hexapod/footTip1',-1,vi.OP_BLOCKING);
    foothold3 = vi.get_object_pose('/hexapod/footTip2',-1,vi.OP_BLOCKING);
    foothold4 = vi.get_object_pose('/hexapod/footTip3',-1,vi.OP_BLOCKING);
    foothold5 = vi.get_object_pose('/hexapod/footTip4',-1,vi.OP_BLOCKING);
    foothold6 = vi.get_object_pose('/hexapod/footTip5',-1,vi.OP_BLOCKING);

    [~,~,linear_foothold1_right] = compute_estimated_velocity(foothold1,last_foothold1,sampling_time);
    [~,~,linear_foothold2_right] = compute_estimated_velocity(foothold2,last_foothold2,sampling_time);
    [~,~,linear_foothold3_right] = compute_estimated_velocity(foothold3,last_foothold3,sampling_time);
    [~,~,linear_foothold4_right] = compute_estimated_velocity(foothold4,last_foothold4,sampling_time);
    [~,~,linear_foothold5_right] = compute_estimated_velocity(foothold5,last_foothold5,sampling_time);
    [~,~,linear_foothold6_right] = compute_estimated_velocity(foothold6,last_foothold6,sampling_time);
    
    linear_velocity_foothold1_right(:,ii) = linear_foothold1_right;
    linear_velocity_foothold2_right(:,ii) = linear_foothold2_right;
    linear_velocity_foothold3_right(:,ii) = linear_foothold3_right;
    linear_velocity_foothold4_right(:,ii) = linear_foothold4_right;
    linear_velocity_foothold5_right(:,ii) = linear_foothold5_right;
    linear_velocity_foothold6_right(:,ii) = linear_foothold6_right;
    %% Send desired values
    if a == 310
        b = 0;
    end
    
    for j = 1:6
    if a <=450
        if corin_q(8+3*(j-1)+1) <= 0.3
            corin_q(8+3*(j-1)+1) = corin_q(8+3*(j-1)+1) + 0.002;
        end
        if corin_q(8+3*(j-1)+2) <= 0
            corin_q(8+3*(j-1)+2) = corin_q(8+3*(j-1)+2) + 0.002;
        end
        if corin_q(8+3*(j-1)+3) >= 0
            corin_q(8+3*(j-1)+3) = corin_q(8+3*(j-1)+3) - 0.005;
        end
    elseif a >=450
        if corin_q(8+3*(j-1)+1) >= 0
            corin_q(8+3*(j-1)+1) = corin_q(8+3*(j-1)+1) - 0.002;
        end
        if corin_q(8+3*(j-1)+2) >= -0.5236
            corin_q(8+3*(j-1)+2) = corin_q(8+3*(j-1)+2) - 0.002;
        end
        if corin_q(8+3*(j-1)+3) <= 2.0942
            corin_q(8+3*(j-1)+3) = corin_q(8+3*(j-1)+3) + 0.005;
        end
    end
    end

    
%     corin_hexapod.send_q_to_vrep(corin_q(9:26));
    corin_hexapod.send_target_joint_to_vrep(corin_q(9:26));
    
    
    last_corin_foothold1 = current_corin_foothold1;
    last_corin_base = current_corin_base;
    last_leg1_q = current_leg1_q;
    last_leg_q = current_leg_q;
    last_foothold1 =foothold1;
    last_foothold2 =foothold2;
    last_foothold3 =foothold3;
    last_foothold4 =foothold4;
    last_foothold5 =foothold5;
    last_foothold6 =foothold6;
    %% Show frames, for testing. This if (and the following else)
    % can be removed for release
    if SHOW_FRAMES
        % Plot current LBR4p end-effector pose on V-REP
        vi.set_object_pose('x1', corin.fkm(corin_q));
        % Plot desired LBR4p end-effector pose on V-REP
        vi.set_object_pose('xd1', lwr4_xd);
        % Plot current youbot end-effector pose on V-REP
        vi.set_object_pose('x2', youbot.fkm(youbot_q));
        % plot desired youbot end-effector pose on V-REP
        vi.set_object_pose('xd2', corin_xd);
        % Show youbot's base frame in V-REP
        vi.set_object_pose('YoubotKinematicBase', ...
            youbot.fkm(youbot_q,1))
        % Show youbot's arm frames on V-REP
    else
%         vi.set_object_pose('x1',DQ(1));
%         vi.set_object_pose('xd1',DQ(1));
%         vi.set_object_pose('x2',DQ(1));
%         vi.set_object_pose('xd2',DQ(1));
%         vi.set_object_pose('YoubotKinematicBase',DQ(1));
%         for k=1:5
%             vi.set_object_pose(corin_reference_frames{k},DQ(1));
%         end
    end
    
    %% get data to plot
    
    ii = ii + 1;
    
    
end
T=0:sampling_time:total_time;


%%
figure(1)
plot(T,Body_vrep_rotation(1,:));
hold on;
plot(T,Body_vrep_rotation(2,:));
hold on;
plot(T,Body_vrep_rotation(3,:));
hold on;
plot(T,Body_vrep_rotation(4,:));
hold on;
plot(T,Body_matlab_rotation(1,:),':');
hold on;
plot(T,Body_matlab_rotation(2,:),':');
hold on;
plot(T,Body_matlab_rotation(3,:),':');
hold on;
plot(T,Body_matlab_rotation(4,:),':');
hold on;
title('base rotation quaternion parameter fkm matlab and vrep')
xlabel('Time/s') 
ylabel('base rotation quaternion parameter fkm matlab and vrep') 
legend('vrep rotation quatertion 1','vrep rotation quatertion 2','vrep rotation quatertion 3','vrep rotation quatertion 4',...
    'matlab rotation quatertion 1','matlab rotation quatertion 2','matlab rotation quatertion 3','matlab rotation quatertion 4');

figure(2)
plot(T,Body_copp_vrep_trans(1,:));
hold on;
plot(T,Body_copp_vrep_trans(2,:));
hold on;
plot(T,Body_copp_vrep_trans(3,:));
hold on;
plot(T,Body_matlab_trans(1,:),':');
hold on;
plot(T,Body_matlab_trans(2,:),':');
hold on;
plot(T,Body_matlab_trans(3,:),':');
hold on;
title('base translation fkm matlab and vrep')
xlabel('Time/s') 
ylabel('base translation fkm matlab and vrep') 
legend('vrep trans x','vrep trans y','vrep trans z','matlab fkm trans x','matlab fkm trans y','matlab fkm trans z');

figure(3)
plot(T,linear_velocity_foothold1_jacobian(1,:));
hold on;
plot(T,linear_velocity_foothold1_jacobian(2,:));
hold on;
plot(T,linear_velocity_foothold1_jacobian(3,:));
hold on;
plot(T,linear_velocity_foothold1_right(1,:),':');
hold on;
plot(T,linear_velocity_foothold1_right(2,:),':');
hold on;
plot(T,linear_velocity_foothold1_right(3,:),':');
hold on;
title('linear velocity foothold1')
xlabel('Time/s') 
ylabel('linear velocity foothold1') 
legend('linear velicity foothold1 jacobian x','linear velicity foothold1 jacobian y','linear velicity foothold1 jacobian z',...
    'linear velicity foothold1 right x','linear velicity foothold1 right y','linear velicity foothold1 right z');

figure(4)
plot(T,linear_velocity_foothold2_jacobian(1,:));
hold on;
plot(T,linear_velocity_foothold2_jacobian(2,:));
hold on;
plot(T,linear_velocity_foothold2_jacobian(3,:));
hold on;
plot(T,linear_velocity_foothold2_right(1,:),':');
hold on;
plot(T,linear_velocity_foothold2_right(2,:),':');
hold on;
plot(T,linear_velocity_foothold2_right(3,:),':');
hold on;
title('linear velocity foothold2')
xlabel('Time/s') 
ylabel('linear velocity foothold2') 
legend('linear velicity foothold2 jacobian x','linear velicity foothold2 jacobian y','linear velicity foothold2 jacobian z',...
    'linear velicity foothold2 right x','linear velicity foothold2 right y','linear velicity foothold2 right z');

figure(5)
plot(T,linear_velocity_foothold3_jacobian(1,:));
hold on;
plot(T,linear_velocity_foothold3_jacobian(2,:));
hold on;
plot(T,linear_velocity_foothold3_jacobian(3,:));
hold on;
plot(T,linear_velocity_foothold3_right(1,:),':');
hold on;
plot(T,linear_velocity_foothold3_right(2,:),':');
hold on;
plot(T,linear_velocity_foothold3_right(3,:),':');
hold on;
title('linear velocity foothold3')
xlabel('Time/s') 
ylabel('linear velocity foothold3') 
legend('linear velicity foothold3 jacobian x','linear velicity foothold3 jacobian y','linear velicity foothold3 jacobian z',...
    'linear velicity foothold3 right x','linear velicity foothold3 right y','linear velicity foothold3 right z');

figure(6)
plot(T,linear_velocity_foothold4_jacobian(1,:));
hold on;
plot(T,linear_velocity_foothold4_jacobian(2,:));
hold on;
plot(T,linear_velocity_foothold4_jacobian(3,:));
hold on;
plot(T,linear_velocity_foothold4_right(1,:),':');
hold on;
plot(T,linear_velocity_foothold4_right(2,:),':');
hold on;
plot(T,linear_velocity_foothold4_right(3,:),':');
hold on;
title('linear velocity foothold4')
xlabel('Time/s') 
ylabel('linear velocity foothold4') 
legend('linear velicity foothold4 jacobian x','linear velicity foothold4 jacobian y','linear velicity foothold4 jacobian z',...
    'linear velicity foothold4 right x','linear velicity foothold4 right y','linear velicity foothold4 right z');

figure(7)
plot(T,linear_velocity_foothold5_jacobian(1,:));
hold on;
plot(T,linear_velocity_foothold5_jacobian(2,:));
hold on;
plot(T,linear_velocity_foothold5_jacobian(3,:));
hold on;
plot(T,linear_velocity_foothold5_right(1,:),':');
hold on;
plot(T,linear_velocity_foothold5_right(2,:),':');
hold on;
plot(T,linear_velocity_foothold5_right(3,:),':');
hold on;
title('linear velocity foothold5')
xlabel('Time/s') 
ylabel('linear velocity foothold5') 
legend('linear velicity foothold5 jacobian x','linear velicity foothold5 jacobian y','linear velicity foothold5 jacobian z',...
    'linear velicity foothold5 right x','linear velicity foothold5 right y','linear velicity foothold5 right z');

figure(8)
plot(T,linear_velocity_foothold6_jacobian(1,:));
hold on;
plot(T,linear_velocity_foothold6_jacobian(2,:));
hold on;
plot(T,linear_velocity_foothold6_jacobian(3,:));
hold on;
plot(T,linear_velocity_foothold6_right(1,:),':');
hold on;
plot(T,linear_velocity_foothold6_right(2,:),':');
hold on;
plot(T,linear_velocity_foothold6_right(3,:),':');
hold on;
title('linear velocity foothold6')
xlabel('Time/s') 
ylabel('linear velocity foothold6') 
legend('linear velicity foothold6 jacobian x','linear velicity foothold6 jacobian y','linear velicity foothold6 jacobian z',...
    'linear velicity foothold6 right x','linear velicity foothold6 right y','linear velicity foothold6 right z');

% figure(6);
% plot(T,Vec_x_ref2foothold_dot(1,:))
% hold on;
% plot(T,Vec_x_ref2foothold_dot_matlab(1,:),':')
% hold on;
% title('first value on the vector foothold1 dot')
% xlabel('Time/s') 
% ylabel('foothold1 dot (1) from vrep and matlab') 
% legend('foothold1 dot 1 vrep','foothold1 dot 1 matlab')
% 
% figure(7);
% plot(T,Vec_x_ref2foothold_dot(2,:))
% hold on;
% plot(T,Vec_x_ref2foothold_dot_matlab(2,:),':')
% hold on;
% title('second value on the vector foothold1 dot')
% xlabel('Time/s') 
% ylabel('foothold1 dot (2) from vrep and matlab') 
% legend('foothold1 dot 2 vrep','foothold1 dot 2 matlab')
% 
% figure(8);
% plot(T,Vec_x_ref2foothold_dot(3,:))
% hold on;
% plot(T,Vec_x_ref2foothold_dot_matlab(3,:),':')
% hold on;
% title('third value on the vector foothold1 dot')
% xlabel('Time/s') 
% ylabel('foothold1 dot (3) from vrep and matlab') 
% legend('foothold1 dot 3 vrep','foothold1 dot 3 matlab')
% 
% figure(9);
% plot(T,Vec_x_ref2foothold_dot(4,:))
% hold on;
% plot(T,Vec_x_ref2foothold_dot_matlab(4,:),':')
% hold on;
% title('forth value on the vector foothold1 dot')
% xlabel('Time/s') 
% ylabel('foothold1 dot (4) from vrep and matlab') 
% legend('foothold1 dot 4 vrep','foothold1 dot 4 matlab')
% 
% figure(10);
% plot(T,Vec_x_ref2foothold_dot(5,:))
% hold on;
% plot(T,Vec_x_ref2foothold_dot_matlab(5,:),':')
% hold on;
% title('fifth value on the vector foothold1 dot')
% xlabel('Time/s') 
% ylabel('foothold1 dot (5) from vrep and matlab') 
% legend('foothold1 dot 5 vrep','foothold1 dot 5 matlab')
% 
% figure(11);
% plot(T,Vec_x_ref2foothold_dot(6,:))
% hold on;
% plot(T,Vec_x_ref2foothold_dot_matlab(6,:),':')
% hold on;
% title('sixth value on the vector foothold1 dot')
% xlabel('Time/s') 
% ylabel('foothold1 dot (6) from vrep and matlab') 
% legend('foothold1 dot 6 vrep','foothold1 dot 6 matlab')
% 
% figure(12);
% plot(T,Vec_x_ref2foothold_dot(7,:))
% hold on;
% plot(T,Vec_x_ref2foothold_dot_matlab(7,:),':')
% hold on;
% title('seventh value on the vector base dot')
% xlabel('Time/s') 
% ylabel('foothold1 dot (7) from vrep and matlab') 
% legend('foothold1 dot 7 vrep','foothold1 dot 7 matlab')
% 
% figure(13);
% plot(T,Vec_x_ref2foothold_dot(8,:))
% hold on;
% plot(T,Vec_x_ref2foothold_dot_matlab(8,:),':')
% hold on;
% title('eighth value on the vector base dot')
% xlabel('Time/s') 
% ylabel('foothold1 dot (8) from vrep and matlab') 
% legend('foothold1 dot 8 vrep','foothold1 dot 8 matlab')

%% End V-REP
vi.stop_simulation();
vi.disconnect();

end



function pose = get_base_from_vrep(vrep_interface,object_name)
base_object_pose = vrep_interface.get_object_pose(object_name);
pose = [0;0;0;0;0;0;0;0];
for i = 1:8
    pose(i) = base_object_pose.q(i);
end
end


function [estimated_velocity,angular_velocity,linear_velocity] = compute_estimated_velocity(current_pose,last_pose,sampling_time)

x_trans = last_pose'* current_pose;
xi = 2*log(x_trans)/ sampling_time;
xi = Ad(last_pose,xi);
angular_velocity_DQ = xi.P;
angular_velocity = vec3(angular_velocity_DQ);
p = last_pose.translation;
linear_velocity_DQ = xi.D-cross(p,angular_velocity);
linear_velocity = vec3(linear_velocity_DQ);
a = norm(angular_velocity);
if a >4
    b = 0
end

estimated_velocity =0.5* xi * current_pose;

% x_trans =  last_pose' * current_pose;
% kxi_1 = 2*log(x_trans)/ sampling_time;
% kxi_2 = 2 * log(last_pose) /sampling_time
% xi = kxi_2 + Ad(last_pose,kxi_1);
% % kxi = Ad(last_pose,kxi_1);
% angular_velocity_DQ = xi.P;
% angular_velocity = vec3(angular_velocity_DQ);
% linear_velocity = vec3(xi.D);
% estimated_velocity =0.5 * xi * current_pose;
end

