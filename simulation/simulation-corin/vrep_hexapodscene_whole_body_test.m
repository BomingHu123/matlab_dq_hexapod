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

function [Foothold1_to_foothold2_trans, Foothold2_to_foothold3_trans, Foothold3_to_foothold4_trans, Foothold4_to_foothold5_trans, Foothold5_to_foothold6_trans,...
    Foothold1_to_foothold2_matlab_trans,Foothold2_to_foothold3_matlab_trans,Foothold3_to_foothold4_matlab_trans,Foothold4_to_foothold5_matlab_trans,...
    Foothold5_to_foothold6_matlab_trans,Body_copp_trans,Body_matlab_trans,Norm_of_error]= vrep_hexapodscene_whole_body_test(simulation_parameters)

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

Foothold1_to_foothold2_trans = zeros(3,max_iterations);
Foothold2_to_foothold3_trans = zeros(3,max_iterations);
Foothold3_to_foothold4_trans = zeros(3,max_iterations);
Foothold4_to_foothold5_trans = zeros(3,max_iterations);
Foothold5_to_foothold6_trans = zeros(3,max_iterations);
Body_copp_trans = zeros(3,max_iterations);

Foothold1_to_foothold2_matlab_trans = zeros(3,max_iterations);
Foothold2_to_foothold3_matlab_trans = zeros(3,max_iterations);
Foothold3_to_foothold4_matlab_trans = zeros(3,max_iterations);
Foothold4_to_foothold5_matlab_trans = zeros(3,max_iterations);
Foothold5_to_foothold6_matlab_trans = zeros(3,max_iterations);
Body_matlab_trans = zeros(3,max_iterations);

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

current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0');
last_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
current_corin_base = vi.get_object_pose('/hexapod');
last_corin_base = vi.get_object_pose('/hexapod/body');
Norm_of_error = [];

corin_q = corin_hexapod.get_q_from_vrep();
current_leg1_q = corin_q(1:3);
last_leg1_q = corin_q(1:3);
ii = 1;


%     foothold1 = corin_hexapod.get_reference_feethold1_from_vrep;
% %     corin.set_reference_to_first_feethold(corin_q);
% 
%     corin.set_reference_to_first_feethold_vrep(foothold1);
    a = 1;
%% Control loop
for t=0:sampling_time:total_time
    %% get information from CoppeliaSim
    vi.synchronous_trigger();
%     pause(0.05);
    
    foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
    foothold2 = vi.get_object_pose('/hexapod/footTip1',-1,vi.OP_BLOCKING);
    foothold3 = vi.get_object_pose('/hexapod/footTip2',-1,vi.OP_BLOCKING);
    foothold4 = vi.get_object_pose('/hexapod/footTip3',-1,vi.OP_BLOCKING);
    foothold5 = vi.get_object_pose('/hexapod/footTip4',-1,vi.OP_BLOCKING);
    foothold6 = vi.get_object_pose('/hexapod/footTip5',-1,vi.OP_BLOCKING);
    body_copp = corin_hexapod.get_body_pose_from_vrep();
    
    foothold1_to_foothold2 = foothold2' * foothold1;
    foothold2_to_foothold3 = foothold3' * foothold2;
    foothold3_to_foothold4 = foothold4' * foothold3;
    foothold4_to_foothold5 = foothold5' * foothold4;
    foothold5_to_foothold6 = foothold6' * foothold5; 
    
    foothold1_to_foothold2_trans4 = vec4(foothold1_to_foothold2.translation);
    foothold1_to_foothold2_trans = foothold1_to_foothold2_trans4(2:4);
    foothold2_to_foothold3_trans4 = vec4(foothold2_to_foothold3.translation);
    foothold2_to_foothold3_trans = foothold2_to_foothold3_trans4(2:4);
    foothold3_to_foothold4_trans4 = vec4(foothold3_to_foothold4.translation);
    foothold3_to_foothold4_trans = foothold3_to_foothold4_trans4(2:4);
    foothold4_to_foothold5_trans4 = vec4(foothold4_to_foothold5.translation);
    foothold4_to_foothold5_trans = foothold4_to_foothold5_trans4(2:4);
    foothold5_to_foothold6_trans4 = vec4(foothold5_to_foothold6.translation);
    foothold5_to_foothold6_trans = foothold5_to_foothold6_trans4(2:4);
    body_copp_trans4 = vec4(body_copp.translation);
    body_copp_trans = body_copp_trans4(2:4);
    
    
    Foothold1_to_foothold2_trans(:,ii) = foothold1_to_foothold2_trans';
    Foothold2_to_foothold3_trans(:,ii) = foothold2_to_foothold3_trans';
    Foothold3_to_foothold4_trans(:,ii) = foothold3_to_foothold4_trans';
    Foothold4_to_foothold5_trans(:,ii) = foothold4_to_foothold5_trans';
    Foothold5_to_foothold6_trans(:,ii) = foothold5_to_foothold6_trans';
    Body_copp_trans(:,ii) = body_copp_trans';
    
    %% get information from Matlab
    foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
%     foothold1 = corin_hexapod.get_reference_feethold1_from_vrep;
%     corin.set_reference_to_first_feethold(corin_q);
    corin.set_reference_to_first_feethold_vrep(foothold1);
    
    corin_q = corin_hexapod.get_q_from_vrep();
    current_leg1_q = corin_q(9:11);
    
%     body_matlab = corin.fkm(corin_q,1);

    
    matlab_fkm_res = corin.fkm(corin_q);
    body_matlab = matlab_fkm_res(1);
    foothold1_to_foothold2_matlab = matlab_fkm_res(2);
    foothold2_to_foothold3_matlab = matlab_fkm_res(3);
    foothold3_to_foothold4_matlab = matlab_fkm_res(4);
    foothold4_to_foothold5_matlab = matlab_fkm_res(5);
    foothold5_to_foothold6_matlab = matlab_fkm_res(6);
    
    
    
    body_matlab_trans4 = vec4(body_matlab.translation);
    body_matlab_trans = body_matlab_trans4(2:4);
    foothold1_to_foothold2_matlab_trans4 = vec4(foothold1_to_foothold2_matlab.translation);
    foothold1_to_foothold2_matlab_trans = foothold1_to_foothold2_matlab_trans4(2:4);
    foothold2_to_foothold3_matlab_trans4 = vec4(foothold2_to_foothold3_matlab.translation);
    foothold2_to_foothold3_matlab_trans = foothold2_to_foothold3_matlab_trans4(2:4);
    foothold3_to_foothold4_matlab_trans4 = vec4(foothold3_to_foothold4_matlab.translation);
    foothold3_to_foothold4_matlab_trans = foothold3_to_foothold4_matlab_trans4(2:4);
    foothold4_to_foothold5_matlab_trans4 = vec4(foothold4_to_foothold5_matlab.translation);
    foothold4_to_foothold5_matlab_trans = foothold4_to_foothold5_matlab_trans4(2:4);
    foothold5_to_foothold6_matlab_trans4 = vec4(foothold5_to_foothold6_matlab.translation);
    foothold5_to_foothold6_matlab_trans = foothold5_to_foothold6_matlab_trans4(2:4);

    
    Foothold1_to_foothold2_matlab_trans(:,ii) = foothold1_to_foothold2_matlab_trans';
    Foothold2_to_foothold3_matlab_trans(:,ii) = foothold2_to_foothold3_matlab_trans';
    Foothold3_to_foothold4_matlab_trans(:,ii) = foothold3_to_foothold4_matlab_trans';
    Foothold4_to_foothold5_matlab_trans(:,ii) = foothold4_to_foothold5_matlab_trans';
    Foothold5_to_foothold6_matlab_trans(:,ii) = foothold5_to_foothold6_matlab_trans';
    Body_matlab_trans(:,ii) = body_matlab_trans';
    
    Coppeliasim_data = [foothold1_to_foothold2,foothold2_to_foothold3,foothold3_to_foothold4,foothold4_to_foothold5,...
        foothold5_to_foothold6,body_copp];
    Matlab_data = [foothold1_to_foothold2_matlab,foothold2_to_foothold3_matlab,foothold3_to_foothold4_matlab,foothold4_to_foothold5_matlab,...
        foothold5_to_foothold6_matlab,body_matlab];
    
    task_error = zeros(48,1);
    for i = 1:6
        foothold = Coppeliasim_data(i);
        foothold_matlab = Matlab_data(i);
        rot_vrep = foothold.rotation;
        rot = foothold_matlab.rotation;
        vec_r = vec4(rot);
        vec_r_vrep = vec4(rot_vrep);
        if norm(vec_r-vec_r_vrep) <= 1e-5
            foothold_matlab_new = foothold_matlab;
        elseif norm(vec_r+vec_r_vrep) <= 1e-5
            rot_new = -rot;
            trans = foothold_matlab.translation;
            foothold_matlab_new =  rot_new +E_*0.5*trans*rot_new;
        else
%             error('error');
        end
        one_task_error = vec8(foothold_matlab_new) - vec8(foothold);
        task_error(8*(i-1) +1 :8*i) = one_task_error;
    end
    
%     footTip0_velocity = compute_estimated_velocity(current_footTip0,last_footTip0,sampling_time);
    
    
    nor = norm(task_error);
    Norm_of_error = [Norm_of_error;nor];


    
%     if corin_q(8+1) <= 0.1
%         corin_q(8+1) = corin_q(8+1) + 0.002;
%     end
%     if corin_q(8+2) <= 0
%         corin_q(8+2) = corin_q(8+2) + 0.002;
%     end
%     if corin_q(8+3) >= 0
%         corin_q(8+3) = corin_q(8+3) - 0.005;
%     end




    a = a+1;
    
    
    %%
    current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
    current_corin_base = vi.get_object_pose('/hexapod/body',-1,vi.OP_BLOCKING);
    [x_ref2f1_dot,angular_velocity_foothold1,linear_velocity_foothold1] = compute_estimated_velocity(current_corin_foothold1,last_corin_foothold1,sampling_time);
    [x_ref2base_dot,~,~] = compute_estimated_velocity(current_corin_base,last_corin_base,sampling_time);
    [linear_velocity_foothold1_vrep,angular_velocity_foothold1_vrep] = vi.get_body_velocity_from_vrep('/hexapod/footTip0');
        [linear_velocity_body_vrep,angular_velocity_body_vrep] = vi.get_body_velocity_from_vrep('/hexapod');
    
    
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
    vec_x_ref2base_dot = vec8(x_ref2base_dot);
    error = vec_x_ref2base_dot_matlab - vec_x_ref2base_dot;
    
    vec_x_ref2foothold1_dot_matlab = matlab_jacobian_2 * leg1_q_dot;
    
    Vec_x_ref2foothold_dot_matlab(:,ii) = vec_x_ref2foothold1_dot_matlab;
    Vec_x_ref2foothold_dot(:,ii) = vec_x_ref2f1_dot;
    
    
    Vec_norm_error(:,ii) = norm(error);
    Vec_x_ref2base_dot_matlab(:,ii) = vec_x_ref2base_dot_matlab;
    Vec_x_ref2base_dot(:,ii) = vec_x_ref2base_dot;
%     
%     DQ_vec_x_ref2base_dot_matlab = DQ(vec_x_ref2base_dot_matlab);
%     estimated_velocity =0.5 * xi * current_pose;
    
    
    if ii == 30
        b =0;
    end
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
figure(1)
plot(T,Foothold1_to_foothold2_trans(1,:))
hold on;
plot(T,Foothold1_to_foothold2_trans(2,:))
hold on;
plot(T,Foothold1_to_foothold2_trans(3,:))
hold on;
plot(T,Foothold1_to_foothold2_matlab_trans(1,:),':')
hold on;
plot(T,Foothold1_to_foothold2_matlab_trans(2,:),':')
hold on;
plot(T,Foothold1_to_foothold2_matlab_trans(3,:),':')
hold on;
title('position')
xlabel('Time/s') 
ylabel('foothold 1 to 2 from vrep and matlab') 
legend('vrep x','vrep y','vrep z',...
    'matlab x','matlab y','matlab z');

figure(2);
plot(T,Vec_x_ref2base_dot_matlab(1,:))
hold on;
plot(T,Vec_x_ref2base_dot_matlab(2,:))
hold on;
plot(T,Vec_x_ref2base_dot_matlab(3,:))
hold on;
plot(T,Vec_x_ref2base_dot_matlab(4,:),':')
hold on;
plot(T,Vec_x_ref2base_dot_matlab(5,:),':')
hold on;
plot(T,Vec_x_ref2base_dot_matlab(6,:),':')
hold on;
plot(T,Vec_x_ref2base_dot_matlab(7,:),':')
hold on;
plot(T,Vec_x_ref2base_dot_matlab(8,:),':')
hold on;
title('position')
xlabel('Time/s') 
ylabel('foothold 1 to 2 from vrep and matlab') 
% legend('vrep x','vrep y','vrep z',...
%     'matlab x','matlab y','matlab z');

figure(3)
plot(T,Angular_velocity_foothold1(1,:))
hold on;
plot(T,Angular_velocity_foothold1(2,:))
hold on;
plot(T,Angular_velocity_foothold1(3,:))
hold on;
plot(T,Angular_velocity_foothold1_Vrep(1,:),':')
hold on;
plot(T,Angular_velocity_foothold1_Vrep(2,:),':')
hold on;
plot(T,Angular_velocity_foothold1_Vrep(3,:),':')
hold on;
title('angular velocity')
xlabel('Time/s') 
ylabel('Angular velocity foothold1') 
legend('matlab rx','matlab ry','matlab rz',...
    'vrep rx','vrep ry','vrep rz');

figure(4)
plot(T,Linear_velocity_foothold1(1,:))
hold on;
plot(T,Linear_velocity_foothold1(2,:))
hold on;
plot(T,Linear_velocity_foothold1(3,:))
hold on;
plot(T,Linear_velocity_foothold1_Vrep(1,:),':')
hold on;
plot(T,Linear_velocity_foothold1_Vrep(2,:),':')
hold on;
plot(T,Linear_velocity_foothold1_Vrep(3,:),':')
hold on;
title('linear velocity')
xlabel('Time/s') 
ylabel('foothold 1 to 2 from vrep and matlab') 
legend('matlab x','matlab y','matlab z',...
    'vrep x','vrep y','vrep z');

figure(5)
plot(T,Linear_velocity_body_Vrep(1,:),':')
hold on;
plot(T,Linear_velocity_body_Vrep(2,:),':')
hold on;
plot(T,Linear_velocity_body_Vrep(3,:),':')
hold on;
title('linear velocity')
xlabel('Time/s') 
ylabel('body from vrep and matlab') 
legend('matlab x','matlab y','matlab z',...
    'vrep x','vrep y','vrep z');

figure(6);
plot(T,Vec_x_ref2base_dot(1,:))
hold on;
plot(T,Vec_x_ref2base_dot_matlab(1,:),':')
hold on;
title('first value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (1) from vrep and matlab') 
legend('base dot 1 vrep','base dot 1 matlab')

figure(7);
plot(T,Vec_x_ref2base_dot(2,:))
hold on;
plot(T,Vec_x_ref2base_dot_matlab(2,:),':')
hold on;
title('second value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (2) from vrep and matlab') 
legend('base dot 2 vrep','base dot 2 matlab')

figure(8);
plot(T,Vec_x_ref2base_dot(3,:))
hold on;
plot(T,Vec_x_ref2base_dot_matlab(3,:),':')
hold on;
title('third value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (3) from vrep and matlab') 
legend('base dot 3 vrep','base dot 3 matlab')

figure(9);
plot(T,Vec_x_ref2base_dot(4,:))
hold on;
plot(T,Vec_x_ref2base_dot_matlab(4,:),':')
hold on;
title('forth value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (4) from vrep and matlab') 
legend('base dot 4 vrep','base dot 4 matlab')

figure(10);
plot(T,Vec_x_ref2base_dot(5,:))
hold on;
plot(T,Vec_x_ref2base_dot_matlab(5,:),':')
hold on;
title('fifth value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (5) from vrep and matlab') 
legend('base dot 5 vrep','base dot 5 matlab')

figure(11);
plot(T,Vec_x_ref2base_dot(6,:))
hold on;
plot(T,Vec_x_ref2base_dot_matlab(6,:),':')
hold on;
title('sixth value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (6) from vrep and matlab') 
legend('base dot 6 vrep','base dot 6 matlab')

figure(12);
plot(T,Vec_x_ref2base_dot(7,:))
hold on;
plot(T,Vec_x_ref2base_dot_matlab(7,:),':')
hold on;
title('seventh value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (7) from vrep and matlab') 
legend('base dot 7 vrep','base dot 7 matlab')

figure(13);
plot(T,Vec_x_ref2base_dot(8,:))
hold on;
plot(T,Vec_x_ref2base_dot_matlab(8,:),':')
hold on;
title('eighth value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (8) from vrep and matlab') 
legend('base dot 8 vrep','base dot 8 matlab')

figure(14);
plot(T,Vec_norm_error);
title('norm of the error for xref2basedot on matlab and vrep')
xlabel('Time/s') 

%%
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

