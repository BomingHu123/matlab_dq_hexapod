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

function vrep_hexapodscene_dqcontroller_dynamic_test(simulation_parameters)

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
corin_hexapod = DQVrepHexapod('Hexapod',vi)
% corin_hexapod = DQVrepCorinHexapod('Corin',vi);
% youbot_vreprobot = YouBotVrepRobot('youBot',vi);

%% Load DQ Robotics kinematics
corin  = corin_hexapod.kinematics();
% youbot = youbot_vreprobot.kinematics();

%% Initialize controllers

solver = DQ_QuadprogSolver;
corin_controller = DQ_ClassicQPController_Hexapod(corin,solver);
corin_controller.set_control_objective(ControlObjective.HexapodTask);
% gain for ju
% corin_controller.set_gain(0.1);
% corin_controller.set_damping(0.05);

% gain for jb
corin_controller.set_gain(20);
corin_controller.set_damping(0.01);

sampling_time = 0.01; % V-REP's sampling time is 50 ms.
total_time = simulation_parameters.total_time; % Total time, in seconds.

%% Set the initial robots configurations
corin_base = vi.get_object_pose('/hexapod/body');
corin_q = corin_hexapod.get_q_from_vrep();
% corin_q = [current_corin_base;  0;0;0;    0;0;0;    0;0;0;     0;0;0;    0;0;0; 0;0;0];


%% Initalize arrays to store all signals resulting from the simulation
max_iterations = round(total_time/sampling_time);

corin_tracking_error_norm = zeros(1,max_iterations);
corin_control_inputs = zeros(corin.get_dim_configuration_space(),...
    max_iterations);
corin_q_vector = zeros(26, ...
    max_iterations);
corin_absolute_error_norm = zeros(1,max_iterations);
corin_relative_error_norm = zeros(1,max_iterations);

corin_velocity_error_norm = zeros(1,max_iterations);

corin_current_foot1 = zeros(8,max_iterations);
corin_current_foot1_trans = zeros(4,max_iterations);
corin_current_foot1_rot = zeros(4,max_iterations);

corin_base_angular_velocity_vrep = zeros(3,max_iterations);
corin_base_angular_velocity_matlab = zeros(3,max_iterations);

corin_base_linear_velocity_vrep = zeros(3,max_iterations);
corin_base_linear_velocity_matlab = zeros(3,max_iterations);

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

corin_angular_velocity_jacobian = zeros(3,max_iterations);
corin_linear_velocity_jacobian = zeros(3,max_iterations);
norm_of_the_erroe_x_ref2_base_dot_matlab_and_vrep =zeros(1,max_iterations);


linear_velocity_foothold1_vrep = zeros(3,max_iterations);
linear_velocity_foothold2_vrep = zeros(3,max_iterations);
linear_velocity_foothold3_vrep = zeros(3,max_iterations);
linear_velocity_foothold4_vrep = zeros(3,max_iterations);
linear_velocity_foothold5_vrep = zeros(3,max_iterations);
linear_velocity_foothold6_vrep = zeros(3,max_iterations);

last_q_dot_vec = zeros(18,max_iterations);
corin_q_vec=zeros(18,max_iterations);
corin_q_plus_u = zeros(18,max_iterations);


% index of the auxiliary arrays used to plot data
ii = 1;
%% Set reference for the manipulator and the mobile manipulator
include_namespace_dq

foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
% corin.set_reference_to_first_feethold(corin_q);
corin.set_reference_to_first_feethold_vrep(foothold1);



% reference_corin_q = [corin_q(1:8); 0.3;0.3;0.3;     0.3;0.3;0.3;     0.3;0.3;0.3;      0.3;0.3;0.3;     0.3;0.3;0.3;  0.3;0.3;0.3;];
% x_ref = corin.fkm(reference_corin_q);
corin_q_task = corin_q;
% corin_q_task(5) = 1.5707;
x_origin = corin.fkm(corin_q_task);
% corin_xd = x_ref;
corin_xd = x_origin;
% r =cos(-pi/12) + k_*sin(-pi/12);
% p = -0.275*i_ + 0.325*j_ + 0.088*k_;
r =cos(0) + j_*sin(0);
p = -0.275*i_ + 0.325*j_ + 0.15*k_;
desired_base_pose = r + E_*0.5*p*r;
corin_xd(1) = desired_base_pose;


% corin_xd(2) = f1tof2';
% corin_xd(3) = f2tof3';
% corin_xd(4) = f3tof4';
% corin_xd(5) = f4tof5';
% corin_xd(6) = f5tof6';


% trans1 = x_origin(2).translation;
% rotation1 = x_origin(2).rotation;
% trans1 = - 1.0262e-05*i_ - 0.2*j_ - 0.05*k_;
% desired_relative_pose1 = rotation1 + E_*0.5*trans1*rotation1;
% corin_xd(2) = desired_relative_pose1;


% Xd= [];
% for i = 1:6
%     x = vec8(corin_xd(i));
%     Xd = [Xd;x];
% end
Xd = corin_xd;



a = 0;
current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0');
last_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0');
current_base = vi.get_object_pose('/hexapod/body');
last_base = vi.get_object_pose('/hexapod/body');
foothold1 = vi.get_object_pose('/hexapod/footTip0');
corin.set_reference_to_first_feethold_vrep(foothold1);

last_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
last_foothold2 = vi.get_object_pose('/hexapod/footTip1',-1,vi.OP_BLOCKING);
last_foothold3 = vi.get_object_pose('/hexapod/footTip2',-1,vi.OP_BLOCKING);
last_foothold4 = vi.get_object_pose('/hexapod/footTip3',-1,vi.OP_BLOCKING);
last_foothold5 = vi.get_object_pose('/hexapod/footTip4',-1,vi.OP_BLOCKING);
last_foothold6 = vi.get_object_pose('/hexapod/footTip5',-1,vi.OP_BLOCKING);

last_leg1_q = corin_q(9:11);
last_leg_q = corin_q(9:26);
%% Control loop
for t=0:sampling_time:total_time
    vi.synchronous_trigger();
    % foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
    %% get q from vrep
%     corin_q = corin_hexapod.get_q_from_vrep();
    current_leg_q = corin_q(9:26);
    leg_q_dot = (current_leg_q - last_leg_q)/sampling_time;  
    %% compute the control input
    corin_u = zeros(18,1);
    for i = 1:6
        corin_u(3*i-2) = 0.1;
        corin_u(3*i-1) = -0.1;
        corin_u(3*i) = 0.2;
    end
    corin_q(9:26) = corin_q(9:26)+corin_u * sampling_time;
    %% Send desired values
    corin_hexapod.send_target_joint_to_vrep(corin_q(9:26));
    %     corin_hexapod.send_q_to_vrep(corin_q(9:26));   
    %% update last value   
    last_leg_q = current_leg_q;   
    %     last_foothold1 =foothold1;
    %     last_foothold2 =foothold2;
    %     last_foothold3 =foothold3;
    %     last_foothold4 =foothold4;
    %     last_foothold5 =foothold5;
    %     last_foothold6 =foothold6;
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
    
    %% Get data to plot them later
    corin_tracking_error_norm(:,ii) = norm(corin_controller.get_last_error_signal());
    last_q_dot_vec(:,ii)=leg_q_dot;
    corin_control_inputs(:,ii) = corin_u;
    corin_q_vector(:,ii) = corin_q;
    task_error = corin_controller.get_last_error_signal();
    corin_absolute_error_norm(:,ii) = norm(task_error(1:8));
    corin_relative_error_norm(:,ii) = norm(task_error(9:48));
    
    %     corin_base_angular_velocity_vrep(:,ii) = angular_base_w_vrep;
    %     corin_base_angular_velocity_matlab(:,ii) = base_angular_velocity;
    ii = ii + 1;
    
    %     corin.set_reference_to_first_feethold(corin_q);
    %     %% set_next_contraints
    %     current_corin_base = get_base_from_vrep(vi,'hexapod');
    %     base_velocity = compute_estimated_base_velocity(current_corin_base,last_corin_base,sampling_time);
    %     [Constraint_matrix,Constraint_Vector] = corin.get_constraint_matrix_and_vector(corin_q,base_velocity);
    %     Constraint_Vector = -Constraint_Vector;
    %     corin_controller.set_equality_constraint(Constraint_matrix,Constraint_Vector);
    %     corin_controller.set_inequality_constraint(Constraint_matrix,Constraint_Vector);
    
    
end
T=0:sampling_time:total_time;

figure(15)
plot(T,corin_control_inputs(1,:))
hold on
plot(T,corin_control_inputs(2,:))
hold on
plot(T,corin_control_inputs(3,:))
hold on
plot(T,last_q_dot_vec(1,:),':')
hold on
plot(T,last_q_dot_vec(2,:),':')
hold on
plot(T,last_q_dot_vec(3,:),':')
hold on
title('control input u for leg1')
xlabel('Time/s')
ylabel('control input u for leg1')
legend('control input u of joint 1 in leg 1','control input u of joint 2 in leg 1','control input u of joint 3 in leg 1',...
    'last qdot of joint 1 in leg1','last qdot of joint 2 in leg1','last qdot of joint 3 in leg1')


figure(16)
plot(T,corin_control_inputs(4,:))
hold on
plot(T,corin_control_inputs(5,:))
hold on
plot(T,corin_control_inputs(6,:))
hold on
plot(T,last_q_dot_vec(4,:),':')
hold on
plot(T,last_q_dot_vec(5,:),':')
hold on
plot(T,last_q_dot_vec(6,:),':')
hold on
title('control input u and last qdot for leg2')
xlabel('Time/s')
ylabel('control input u for leg2')
legend('control input u of joint 1 in leg 2','control input u of joint 2 in leg 2','control input u of joint 3 in leg 2',...
    'last qdot of joint 1 in leg2','last qdot of joint 2 in leg2','last qdot of joint 3 in leg2')

figure(17)
plot(T,corin_control_inputs(7,:))
hold on
plot(T,corin_control_inputs(8,:))
hold on
plot(T,corin_control_inputs(9,:))
hold on
plot(T,last_q_dot_vec(7,:),':')
hold on
plot(T,last_q_dot_vec(8,:),':')
hold on
plot(T,last_q_dot_vec(9,:),':')
hold on
title('control input u for leg3')
xlabel('Time/s')
ylabel('control input u for leg3')
legend('control input u of joint 1 in leg 3','control input u of joint 2 in leg 3','control input u of joint 3 in leg 3',...
    'last qdot of joint 1 in leg3','last qdot of joint 2 in leg3','last qdot of joint 3 in leg3')

figure(18)
plot(T,corin_control_inputs(10,:))
hold on
plot(T,corin_control_inputs(11,:))
hold on
plot(T,corin_control_inputs(12,:))
hold on
plot(T,last_q_dot_vec(10,:),':')
hold on
plot(T,last_q_dot_vec(11,:),':')
hold on
plot(T,last_q_dot_vec(12,:),':')
hold on
title('control input u for leg4')
xlabel('Time/s')
ylabel('control input u for leg1')
legend('control input u of joint 1 in leg 4','control input u of joint 2 in leg 4','control input u of joint 3 in leg 4',...
    'last qdot of joint 1 in leg 4','last qdot of joint 2 in leg 4','last qdot of joint 3 in leg 4')

figure(19)
plot(T,corin_control_inputs(13,:))
hold on
plot(T,corin_control_inputs(14,:))
hold on
plot(T,corin_control_inputs(15,:))
hold on
plot(T,last_q_dot_vec(13,:),':')
hold on
plot(T,last_q_dot_vec(14,:),':')
hold on
plot(T,last_q_dot_vec(15,:),':')
hold on
title('control input u for leg5')
xlabel('Time/s')
ylabel('control input u for leg5')
legend('control input u of joint 1 in leg 5','control input u of joint 2 in leg 5','control input u of joint 3 in leg 5',...
    'last qdot of joint 1 in leg 5','last qdot of joint 2 in leg 5','last qdot of joint 3 in leg 5')

figure(20)
plot(T,corin_control_inputs(16,:))
hold on
plot(T,corin_control_inputs(17,:))
hold on
plot(T,corin_control_inputs(18,:))
hold on
plot(T,last_q_dot_vec(16,:),':')
hold on
plot(T,last_q_dot_vec(17,:),':')
hold on
plot(T,last_q_dot_vec(18,:),':')
hold on
title('control input u for leg6')
xlabel('Time/s')
ylabel('control input u for leg6')
legend('control input u of joint 1 in leg 6','control input u of joint 2 in leg 6','control input u of joint 3 in leg 6',...
    'last qdot of joint 1 in leg 6','last qdot of joint 2 in leg 6','last qdot of joint 3 in leg 6')
% 
% figure(21)
% plot(T,corin_q_vec(1,:))
% hold on
% plot(T,corin_q_vec(2,:))
% hold on
% plot(T,corin_q_vec(3,:))
% hold on
% plot(T,corin_q_plus_u(1,:),':')
% hold on
% plot(T,corin_q_plus_u(2,:),':')
% hold on
% plot(T,corin_q_plus_u(3,:),':')
% hold on
% title('corin q')
% xlabel('Time/s')
% ylabel('corin q')
% legend('q of joint 1 in leg 1 read from vrep','q of joint 2 in leg 1 read from vrep','q of joint 3 in leg 3 read from vrep',...
%     'q of joint 1 in leg 1 transport to vrep','q of joint 2 in leg 1 transport to vrep','q of joint 3 in leg 1 transport to vrep')
% 
% figure(21)
% plot(T,corin_q_vec(1,:))
% hold on
% plot(T,corin_q_vec(2,:))
% hold on
% plot(T,corin_q_vec(3,:))
% hold on
% plot(T,corin_q_plus_u(1,:),':')
% hold on
% plot(T,corin_q_plus_u(2,:),':')
% hold on
% plot(T,corin_q_plus_u(3,:),':')
% hold on
% title('corin q')
% xlabel('Time/s')
% ylabel('corin q')
% legend('q of joint 1 in leg 1 read from vrep','q of joint 2 in leg 1 read from vrep','q of joint 3 in leg 2 read from vrep',...
%     'q of joint 1 in leg 1 transport to vrep','q of joint 2 in leg 1 transport to vrep','q of joint 3 in leg 1 transport to vrep')
% 
% figure(22)
% plot(T,corin_q_vec(4,:))
% hold on
% plot(T,corin_q_vec(5,:))
% hold on
% plot(T,corin_q_vec(6,:))
% hold on
% plot(T,corin_q_plus_u(4,:),':')
% hold on
% plot(T,corin_q_plus_u(5,:),':')
% hold on
% plot(T,corin_q_plus_u(6,:),':')
% hold on
% title('corin q')
% xlabel('Time/s')
% ylabel('corin q')
% legend('q of joint 1 in leg 2 read from vrep','q of joint 2 in leg 2 read from vrep','q of joint 3 in leg 2 read from vrep',...
%     'q of joint 1 in leg 2 transport to vrep','q of joint 2 in leg 2 transport to vrep','q of joint 3 in leg 2 transport to vrep')
% 
% figure(23)
% plot(T,corin_q_vec(7,:))
% hold on
% plot(T,corin_q_vec(8,:))
% hold on
% plot(T,corin_q_vec(9,:))
% hold on
% plot(T,corin_q_plus_u(7,:),':')
% hold on
% plot(T,corin_q_plus_u(8,:),':')
% hold on
% plot(T,corin_q_plus_u(9,:),':')
% hold on
% title('corin q')
% xlabel('Time/s')
% ylabel('corin q')
% legend('q of joint 1 in leg 3 read from vrep','q of joint 2 in leg 3 read from vrep','q of joint 3 in leg 3 read from vrep',...
%     'q of joint 1 in leg 3 transport to vrep','q of joint 2 in leg 3 transport to vrep','q of joint 3 in leg 3 transport to vrep')
% 
% figure(24)
% plot(T,corin_q_vec(10,:))
% hold on
% plot(T,corin_q_vec(11,:))
% hold on
% plot(T,corin_q_vec(12,:))
% hold on
% plot(T,corin_q_plus_u(10,:),':')
% hold on
% plot(T,corin_q_plus_u(11,:),':')
% hold on
% plot(T,corin_q_plus_u(12,:),':')
% hold on
% title('corin q')
% xlabel('Time/s')
% ylabel('corin q')
% legend('q of joint 1 in leg 4 read from vrep','q of joint 2 in leg 4 read from vrep','q of joint 3 in leg 4 read from vrep',...
%     'q of joint 1 in leg 4 transport to vrep','q of joint 2 in leg 4 transport to vrep','q of joint 3 in leg 4 transport to vrep')
% 
% figure(25)
% plot(T,corin_q_vec(13,:))
% hold on
% plot(T,corin_q_vec(14,:))
% hold on
% plot(T,corin_q_vec(15,:))
% hold on
% plot(T,corin_q_plus_u(13,:),':')
% hold on
% plot(T,corin_q_plus_u(14,:),':')
% hold on
% plot(T,corin_q_plus_u(15,:),':')
% hold on
% title('corin q')
% xlabel('Time/s')
% ylabel('corin q')
% legend('q of joint 1 in leg 5 read from vrep','q of joint 2 in leg 5 read from vrep','q of joint 3 in leg 5 read from vrep',...
%     'q of joint 1 in leg 5 transport to vrep','q of joint 2 in leg 5 transport to vrep','q of joint 3 in leg 5 transport to vrep')
% 
% figure(26)
% plot(T,corin_q_vec(16,:))
% hold on
% plot(T,corin_q_vec(17,:))
% hold on
% plot(T,corin_q_vec(18,:))
% hold on
% plot(T,corin_q_plus_u(16,:),':')
% hold on
% plot(T,corin_q_plus_u(17,:),':')
% hold on
% plot(T,corin_q_plus_u(18,:),':')
% hold on
% title('corin q')
% xlabel('Time/s')
% ylabel('corin q')
% legend('q of joint 1 in leg 6 read from vrep','q of joint 2 in leg 6 read from vrep','q of joint 3 in leg 6 read from vrep',...
%     'q of joint 1 in leg 6 transport to vrep','q of joint 2 in leg 6 transport to vrep','q of joint 3 in leg 6 transport to vrep')

% figure(9)
% plot(T,linear_velocity_foothold1_jacobian(1,:));
% hold on;
% plot(T,linear_velocity_foothold1_jacobian(2,:));
% hold on;
% plot(T,linear_velocity_foothold1_jacobian(3,:));
% hold on;
% plot(T,linear_velocity_foothold1_right(1,:),':');
% hold on;
% plot(T,linear_velocity_foothold1_right(2,:),':');
% hold on;
% plot(T,linear_velocity_foothold1_right(3,:),':');
% hold on;
% title('linear velocity foothold1')
% xlabel('Time/s')
% ylabel('linear velocity foothold1')
% legend('linear velicity foothold1 jacobian x','linear velicity foothold1 jacobian y','linear velicity foothold1 jacobian z',...
%     'linear velicity foothold1 right x','linear velicity foothold1 right y','linear velicity foothold1 right z');
%
% figure(10)
% plot(T,linear_velocity_foothold2_jacobian(1,:));
% hold on;
% plot(T,linear_velocity_foothold2_jacobian(2,:));
% hold on;
% plot(T,linear_velocity_foothold2_jacobian(3,:));
% hold on;
% plot(T,linear_velocity_foothold2_right(1,:),':');
% hold on;
% plot(T,linear_velocity_foothold2_right(2,:),':');
% hold on;
% plot(T,linear_velocity_foothold2_right(3,:),':');
% hold on;
% title('linear velocity foothold2')
% xlabel('Time/s')
% ylabel('linear velocity foothold2')
% legend('linear velicity foothold2 jacobian x','linear velicity foothold2 jacobian y','linear velicity foothold2 jacobian z',...
%     'linear velicity foothold2 right x','linear velicity foothold2 right y','linear velicity foothold2 right z');
%
% figure(11)
% plot(T,linear_velocity_foothold3_jacobian(1,:));
% hold on;
% plot(T,linear_velocity_foothold3_jacobian(2,:));
% hold on;
% plot(T,linear_velocity_foothold3_jacobian(3,:));
% hold on;
% plot(T,linear_velocity_foothold3_right(1,:),':');
% hold on;
% plot(T,linear_velocity_foothold3_right(2,:),':');
% hold on;
% plot(T,linear_velocity_foothold3_right(3,:),':');
% hold on;
% title('linear velocity foothold3')
% xlabel('Time/s')
% ylabel('linear velocity foothold3')
% legend('linear velicity foothold3 jacobian x','linear velicity foothold3 jacobian y','linear velicity foothold3 jacobian z',...
%     'linear velicity foothold3 right x','linear velicity foothold3 right y','linear velicity foothold3 right z');
%
% figure(12)
% plot(T,linear_velocity_foothold4_jacobian(1,:));
% hold on;
% plot(T,linear_velocity_foothold4_jacobian(2,:));
% hold on;
% plot(T,linear_velocity_foothold4_jacobian(3,:));
% hold on;
% plot(T,linear_velocity_foothold4_right(1,:),':');
% hold on;
% plot(T,linear_velocity_foothold4_right(2,:),':');
% hold on;
% plot(T,linear_velocity_foothold4_right(3,:),':');
% hold on;
% title('linear velocity foothold4')
% xlabel('Time/s')
% ylabel('linear velocity foothold4')
% legend('linear velicity foothold4 jacobian x','linear velicity foothold4 jacobian y','linear velicity foothold4 jacobian z',...
%     'linear velicity foothold4 right x','linear velicity foothold4 right y','linear velicity foothold4 right z');
%
% figure(13)
% plot(T,linear_velocity_foothold5_jacobian(1,:));
% hold on;
% plot(T,linear_velocity_foothold5_jacobian(2,:));
% hold on;
% plot(T,linear_velocity_foothold5_jacobian(3,:));
% hold on;
% plot(T,linear_velocity_foothold5_right(1,:),':');
% hold on;
% plot(T,linear_velocity_foothold5_right(2,:),':');
% hold on;
% plot(T,linear_velocity_foothold5_right(3,:),':');
% hold on;
% title('linear velocity foothold5')
% xlabel('Time/s')
% ylabel('linear velocity foothold5')
% legend('linear velicity foothold5 jacobian x','linear velicity foothold5 jacobian y','linear velicity foothold5 jacobian z',...
%     'linear velicity foothold5 right x','linear velicity foothold5 right y','linear velicity foothold5 right z');
%
% figure(14)
% plot(T,linear_velocity_foothold6_jacobian(1,:));
% hold on;
% plot(T,linear_velocity_foothold6_jacobian(2,:));
% hold on;
% plot(T,linear_velocity_foothold6_jacobian(3,:));
% hold on;
% plot(T,linear_velocity_foothold6_right(1,:),':');
% hold on;
% plot(T,linear_velocity_foothold6_right(2,:),':');
% hold on;
% plot(T,linear_velocity_foothold6_right(3,:),':');
% hold on;
% title('linear velocity foothold6')
% xlabel('Time/s')
% ylabel('linear velocity foothold6')
% legend('linear velicity foothold6 jacobian x','linear velicity foothold6 jacobian y','linear velicity foothold6 jacobian z',...
%     'linear velicity foothold6 right x','linear velicity foothold6 right y','linear velicity foothold6 right z');
%
% figure(15)
% plot(T,linear_velocity_foothold1_right(1,:));
% hold on;
% plot(T,linear_velocity_foothold1_right(2,:));
% hold on;
% plot(T,linear_velocity_foothold1_right(3,:));
% hold on;
% title('linear velocity foothold1')
% xlabel('Time/s')
% ylabel('linear velocity foothold1')
% legend('linear velicity foothold1 right x','linear velicity foothold1 right y','linear velicity foothold1 right z');
%
% figure(16)
% plot(T,linear_velocity_foothold2_right(1,:));
% hold on;
% plot(T,linear_velocity_foothold2_right(2,:));
% hold on;
% plot(T,linear_velocity_foothold2_right(3,:));
% hold on;
% title('linear velocity foothold2')
% xlabel('Time/s')
% ylabel('linear velocity foothold2')
% legend('linear velicity foothold2 right x','linear velicity foothold2 right y','linear velicity foothold2 right z');
%
% figure(17)
% plot(T,linear_velocity_foothold3_right(1,:));
% hold on;
% plot(T,linear_velocity_foothold3_right(2,:));
% hold on;
% plot(T,linear_velocity_foothold3_right(3,:));
% hold on;
% title('linear velocity foothold3')
% xlabel('Time/s')
% ylabel('linear velocity foothold3')
% legend('linear velicity foothold3 right x','linear velicity foothold3 right y','linear velicity foothold3 right z');
%
% figure(18)
% plot(T,linear_velocity_foothold4_right(1,:));
% hold on;
% plot(T,linear_velocity_foothold4_right(2,:));
% hold on;
% plot(T,linear_velocity_foothold4_right(3,:));
% hold on;
% title('linear velocity foothold4')
% xlabel('Time/s')
% ylabel('linear velocity foothold4')
% legend('linear velicity foothold4 right x','linear velicity foothold4 right y','linear velicity foothold4 right z');
%
% figure(19)
% plot(T,linear_velocity_foothold5_right(1,:));
% hold on;
% plot(T,linear_velocity_foothold5_right(2,:));
% hold on;
% plot(T,linear_velocity_foothold5_right(3,:));
% hold on;
% title('linear velocity foothold5')
% xlabel('Time/s')
% ylabel('linear velocity foothold5')
% legend('linear velicity foothold5 right x','linear velicity foothold5 right y','linear velicity foothold5 right z');
%
% figure(20)
% plot(T,linear_velocity_foothold6_right(1,:));
% hold on;
% plot(T,linear_velocity_foothold6_right(2,:));
% hold on;
% plot(T,linear_velocity_foothold6_right(3,:));
% hold on;
% plot(T,linear_velocity_foothold6_vrep(3,:),':');
% hold on;
% plot(T,linear_velocity_foothold6_vrep(3,:),':');
% hold on;
% plot(T,linear_velocity_foothold6_vrep(3,:),':');
% hold on;
% title('linear velocity foothold6')
% xlabel('Time/s')
% ylabel('linear velocity foothold6')
% legend( 'linear velicity foothold6 approximation x','linear velicity foothold6 approximation y','linear velicity foothold6 approximation z',...
%      'linear velicity foothold6 vrep x','linear velicity foothold6 vrep y','linear velicity foothold6 vrep z');
%
% figure(21)
% plot(T,linear_velocity_foothold1_vrep(1,:));
% hold on;
% plot(T,linear_velocity_foothold1_vrep(2,:));
% hold on;
% plot(T,linear_velocity_foothold1_vrep(3,:));
% hold on;
% title('linear velocity foothold1')
% xlabel('Time/s')
% ylabel('linear velocity foothold1')
% legend( 'linear velicity foothold1 vrep x','linear velicity foothold1 vrep y','linear velicity foothold1 vrep z');
%
% figure(22)
% plot(T,linear_velocity_foothold2_vrep(1,:));
% hold on;
% plot(T,linear_velocity_foothold2_vrep(2,:));
% hold on;
% plot(T,linear_velocity_foothold2_vrep(3,:));
% hold on;
% title('linear velocity foothold2')
% xlabel('Time/s')
% ylabel('linear velocity foothold2')
% legend( 'linear velicity foothold2 vrep x','linear velicity foothold2 vrep y','linear velicity foothold2 vrep z');
% figure(23)
% plot(T,linear_velocity_foothold3_vrep(1,:));
% hold on;
% plot(T,linear_velocity_foothold3_vrep(2,:));
% hold on;
% plot(T,linear_velocity_foothold3_vrep(3,:));
% hold on;
% title('linear velocity foothold3')
% xlabel('Time/s')
% ylabel('linear velocity foothold3')
% legend( 'linear velicity foothold3 vrep x','linear velicity foothold3 vrep y','linear velicity foothold3 vrep z');
% figure(24)
% plot(T,linear_velocity_foothold4_vrep(1,:));
% hold on;
% plot(T,linear_velocity_foothold4_vrep(2,:));
% hold on;
% plot(T,linear_velocity_foothold4_vrep(3,:));
% hold on;
% title('linear velocity foothold4')
% xlabel('Time/s')
% ylabel('linear velocity foothold4')
% legend( 'linear velicity foothold4 vrep x','linear velicity foothold4 vrep y','linear velicity foothold4 vrep z');
% figure(25)
% plot(T,linear_velocity_foothold5_vrep(1,:));
% hold on;
% plot(T,linear_velocity_foothold5_vrep(2,:));
% hold on;
% plot(T,linear_velocity_foothold5_vrep(3,:));
% hold on;
% title('linear velocity foothold5')
% xlabel('Time/s')
% ylabel('linear velocity foothold5')
% legend( 'linear velicity foothold5 vrep x','linear velicity foothold5 vrep y','linear velicity foothold5 vrep z');
% figure(26)
% plot(T,linear_velocity_foothold6_vrep(1,:));
% hold on;
% plot(T,linear_velocity_foothold6_vrep(2,:));
% hold on;
% plot(T,linear_velocity_foothold6_vrep(3,:));
% hold on;
% title('linear velocity foothold6')
% xlabel('Time/s')
% ylabel('linear velocity foothold6')
% legend( 'linear velicity foothold6 vrep x','linear velicity foothold6 vrep y','linear velicity foothold6 vrep z');


%----------------------------------------------------------------
% figure(6)
% plot(T,corin_foothold1_pose_vrep(1,:))
% hold on;
% plot(T,corin_foothold1_pose_vrep(2,:))
% hold on;
% plot(T,corin_foothold1_pose_vrep(3,:))
% hold on;
% title('pose of foothold1')
% xlabel('Time/s')
% ylabel('velocity from vrep and matlab')
% legend('foothold1 pose from vrep x','foothold1 pose from vrep y','foothold1 pose from vrep z')
%
% figure(7)
% plot(T,corin_foothold2_pose_vrep(1,:))
% hold on;
% plot(T,corin_foothold2_pose_vrep(2,:))
% hold on;
% plot(T,corin_foothold2_pose_vrep(3,:))
% hold on;
% title('pose of foothold2')
% xlabel('Time/s')
% ylabel('velocity from vrep and matlab')
% legend('foothold2 pose from vrep x','foothold2 pose from vrep y','foothold2 pose from vrep z')
%
% figure(8)
% plot(T,corin_foothold3_pose_vrep(1,:))
% hold on;
% plot(T,corin_foothold3_pose_vrep(2,:))
% hold on;
% plot(T,corin_foothold3_pose_vrep(3,:))
% hold on;
% title('pose of foothold3')
% xlabel('Time/s')
% ylabel('velocity from vrep and matlab')
% legend('foothold3 pose from vrep x','foothold3 pose from vrep y','foothold3 pose from vrep z')
%
% figure(9)
% plot(T,corin_foothold4_pose_vrep(1,:))
% hold on;
% plot(T,corin_foothold4_pose_vrep(2,:))
% hold on;
% plot(T,corin_foothold4_pose_vrep(3,:))
% hold on;
% title('pose of foothold4')
% xlabel('Time/s')
% ylabel('velocity from vrep and matlab')
% legend('foothold4 pose from vrep x','foothold4 pose from vrep y','foothold4 pose from vrep z')
%
% figure(10)
% plot(T,corin_foothold5_pose_vrep(1,:))
% hold on;
% plot(T,corin_foothold5_pose_vrep(2,:))
% hold on;
% plot(T,corin_foothold5_pose_vrep(3,:))
% hold on;
% title('pose of foothold5')
% xlabel('Time/s')
% ylabel('velocity from vrep and matlab')
% legend('foothold5 pose from vrep x','foothold5 pose from vrep y','foothold5 pose from vrep z')
%
% figure(11)
% plot(T,corin_foothold6_pose_vrep(1,:))
% hold on;
% plot(T,corin_foothold6_pose_vrep(2,:))
% hold on;
% plot(T,corin_foothold6_pose_vrep(3,:))
% hold on;
% title('pose of foothold6')
% xlabel('Time/s')
% ylabel('velocity from vrep and matlab')
% legend('foothold6 pose from vrep x','foothold6 pose from vrep y','foothold6 pose from vrep z')

%% End V-REP
vi.stop_simulation();
vi.disconnect();

end

function line = get_line_from_vrep(vrep_interface,object_name,primitive)
line_object_pose = vrep_interface.get_object_pose(object_name);
p = translation(line_object_pose);
r = rotation(line_object_pose);
l = Ad(r,primitive);
m = cross(p,l);
line = l + DQ.E*m;
end

function plane = get_plane_from_vrep(vrep_interface,object_name,primitive)
plane_object_pose = vrep_interface.get_object_pose(object_name);
p = translation(plane_object_pose);
r = rotation(plane_object_pose);
n = Ad(r,primitive);
d = dot(p,n);
plane = n + DQ.E*d;
end

function pose = get_base_from_vrep(vrep_interface,object_name)
pose = vrep_interface.get_object_pose(object_name);
% pose = [0;0;0;0;0;0;0;0];
% for i = 1:8
%     pose(i) = base_object_pose.q(i);
% end
end


function [youbot_xd, youbot_ff] = compute_youbot_reference(controller, ...
    lbr4p_xd, lbr4p_ff)
persistent tc;
persistent rcircle;
persistent first_iteration;

include_namespace_dq;

circle_radius = 0.1;
tcircle = 1 + E_ * 0.5 * circle_radius * j_;

%% Youbot trajectory
% Those are the trajectory components to track the whiteboard
youbot_xd = lbr4p_xd * (1 + 0.5*E_*0.015*k_) * j_;
youbot_ff = lbr4p_ff * (1 + 0.5*E_*0.015*k_) * j_;
% Now we modify the trajectory in order to draw a circle, but we do it
% only if the whiteboard pen tip is on the whiteboard surface.
if isempty(first_iteration)
    first_iteration = false;
    tc = 0;
    rcircle = DQ(1);
elseif norm(controller.get_last_error_signal()) < 0.002
    tc = tc + 0.1; % Increment around 0.5 deg.
    rcircle = cos(tc/2) + k_ * sin(tc/2);
end
youbot_xd = youbot_xd * rcircle * tcircle;
youbot_ff = youbot_ff * rcircle * tcircle;
end


function [Jconstraint, bconstraint] = compute_constraints(youbot, youbot_q, plane, cylinder1, cylinder2)
% We define differential inequalities given by d_dot >= -eta*d,
% where d is the distance from the end-effector to the
% primitive, d_dot is its time derivative and eta determines the
% maximum rate for the approach velocity.

% This information should be inside the robot itself
robot_radius = 0.35;
radius_cylinder1 = 0.1;
radius_cylinder2 = 0.1;

include_namespace_dq

% Get base translation and translation Jacobian
youbot_base = youbot.get_chain(1);
youbot_base_pose = youbot_base.raw_fkm(youbot_q);
Jx = youbot_base.raw_pose_jacobian(youbot_q);
t = translation(youbot_base_pose);
Jt = [youbot.translation_jacobian(Jx,youbot_base_pose),zeros(4,5)];

% First we calculate the primitives for the plane
Jdist_plane = youbot.point_to_plane_distance_jacobian(Jt, t, plane);
dist_plane = DQ_Geometry.point_to_plane_distance(t,plane) - robot_radius;

% Now we compute the primitives for the two cylinders
Jdist_cylinder1 = youbot.point_to_line_distance_jacobian(Jt, t, cylinder1);
dist_cylinder1 = DQ_Geometry.point_to_line_squared_distance(t,cylinder1) - ...
    (radius_cylinder1 + robot_radius)^2;

Jdist_cylinder2 = youbot.point_to_line_distance_jacobian(Jt, t, cylinder2);
dist_cylinder2 = DQ_Geometry.point_to_line_squared_distance(t,cylinder2) - ...
    (radius_cylinder2 + robot_radius)^2;

% Assemble all constraint matrices into a unique matrix
Jconstraint = [Jdist_plane; Jdist_cylinder1; Jdist_cylinder2];
% And do analagously to the constraint vectors
bconstraint = [dist_plane; dist_cylinder1; dist_cylinder2];
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

function [xi,angular_velocity, linear_velocity] = compute_velocity_from_xdot(xdot,current_pose)
xi = 2*xdot *current_pose';
angular_velocity_DQ = xi.P;
angular_velocity = vec3(angular_velocity_DQ);
p = current_pose.translation;
linear_velocity_DQ = xi.D-cross(p,angular_velocity);
linear_velocity = vec3(linear_velocity_DQ);
end