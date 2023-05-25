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

function vrep_hexapodscene_dqcontroller_without_constraint_test_jacobian(simulation_parameters)

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
corin_controller.set_gain(1);
corin_controller.set_damping(0.01);

sampling_time = 0.0001; % V-REP's sampling time is 50 ms.
total_time = simulation_parameters.total_time; % Total time, in seconds.

%% Set the initial robots configurations
corin_base = vi.get_object_pose('/hexapod/body',-1,vi.OP_BLOCKING);
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

function_norm = zeros(1,max_iterations);

corin_current_foot1 = zeros(8,max_iterations);
corin_current_foot1_trans = zeros(4,max_iterations);
corin_current_foot1_rot = zeros(4,max_iterations);

corin_angular_velocity_vrep = zeros(3,max_iterations);
corin_angular_velocity_matlab = zeros(3,max_iterations);

corin_linear_velocity_vrep = zeros(3,max_iterations);
corin_linear_velocity_matlab = zeros(3,max_iterations);

corin_foothold1_linear_velocity = zeros(3,max_iterations);
corin_foothold1_angular_velocity = zeros(3,max_iterations);

corin_angular_velocity_jacobian = zeros(3,max_iterations);
corin_linear_velocity_jacobian = zeros(3,max_iterations);

corin_base_cot_matlab = zeros(8,max_iterations);
corin_base_cot_matlab_2 = zeros(8,max_iterations);
corin_base_cot_vrep = zeros(8,max_iterations);
corin_norm_error_dot = zeros(1,max_iterations);

% index of the auxiliary arrays used to plot data
ii = 1;
%% Set reference for the manipulator and the mobile manipulator
include_namespace_dq

foothold1 = corin_hexapod.get_reference_feethold1_from_vrep;
% corin.set_reference_to_first_feethold(corin_q);
corin.set_reference_to_first_feethold_vrep(foothold1);
joint1andjoint3 = corin_hexapod.get_object_pose_from_vrep();
joint1andjoint2 = corin_hexapod.get_object_pose_from_vrep2();
joint2andjoint3 = corin_hexapod.get_object_pose_from_vrep3();
joint1pose = corin_hexapod.get_object_pose_from_vrep_world1();
joint3pose = corin_hexapod.get_object_pose_from_vrep_world3();
joint1tojoint3 = joint1pose' * joint3pose;
joint1tojoint3_v2 = joint1andjoint2*joint2andjoint3;

world3v1 = joint1pose* joint1tojoint3
world3v2= joint1pose * joint1tojoint3_v2


x_origin = corin.fkm(corin_q);
corin_xd = x_origin;
% r =cos(-pi/12) + k_*sin(-pi/12);
% p = -0.275*i_ + 0.325*j_ + 0.088*k_;
r =cos(0) + k_*sin(0);
p = -0.275*i_ + 0.325*j_ + 0.15*k_;
desired_base_pose = r + E_*0.5*p*r;
corin_xd(1) = desired_base_pose;


% trans1 = x_origin(2).translation;
% rotation1 = x_origin(2).rotation;
% trans1 = - 1.0262e-05*i_ - 0.2*j_ - 0.05*k_;
% desired_relative_pose1 = rotation1 + E_*0.5*trans1*rotation1;
% corin_xd(2) = desired_relative_pose1;


Xd= [];
for i = 1:6
    x = vec8(corin_xd(i));
    Xd = [Xd;x];
end


% % % first calculate the constraints
% base_velocity = compute_estimated_velocity(corin_base,corin_base,sampling_time);
% % vec_base = DQ(0);
% [Constraint_matrix,Constraint_Vector] = corin.get_constraint_matrix_and_vector(corin_q,base_velocity);
% Constraint_Vector = -Constraint_Vector;
% % control.set_inequality_constraint(Constraint_matrix,Constraint_Vector);
% corin_controller.set_equality_constraint(Constraint_matrix,Constraint_Vector);
a = 0;
% 
current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
last_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);

current_base = vi.get_object_pose('/hexapod/body',-1,vi.OP_BLOCKING);
last_base = vi.get_object_pose('/hexapod/body',-1,vi.OP_BLOCKING);
current_leg1_q = corin_q(1:3);
last_leg1_q = corin_q(1:3);
%     current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0');
%     corin.set_reference_to_first_feethold_vrep(current_corin_foothold1);
%% Control loop
for t=0:sampling_time:total_time
    vi.synchronous_trigger();
% pause(0.01)
% x_current = corin.fkm(corin_q);
% for i = 2:6
%     x = vec8(x_current(i));
%     Xd(8*(i-1)+1:8*i) = x;
% end
    %% get data from vrep
    a = a+1;
    foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
%     foothold1 = corin_hexapod.get_reference_feethold1_from_vrep;
    corin.set_reference_to_first_feethold_vrep(foothold1);
    corin_q = corin_hexapod.get_q_from_vrep();
    current_leg1_q = corin_q(9:11);
%% set constraint
% current_corin_base = get_base_from_vrep(vi,'/hexapod/body');
% base_velocity = compute_estimated_velocity(current_corin_base,last_corin_base,sampling_time);
%     [Constraint_matrix,Constraint_Vector] = corin.get_constraint_matrix_and_vector(corin_q,base_velocity);
% %     Constraint_Vector = Constraint_Vector;
% 
%     % control.set_inequality_constraint(Constraint_matrix,Constraint_Vector);
%     corin_controller.set_equality_constraint(Constraint_matrix,Constraint_Vector);

current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);

% get_base_from_vrep(vi,'/hexapod/footTip0');
% current_footTip0 = vi.get_object_pose('/hexapod/footTip0');
current_base_trans = current_corin_foothold1.translation;
current_base_rot = current_corin_foothold1.rotation;

[x_ref2f1_dot,angular_foothold1_velocity,linear_foothold1_velocity] = compute_estimated_velocity(current_corin_foothold1,last_corin_foothold1,sampling_time);

% foothold1_velocity = compute_estimated_velocity(current_corin_foothold1,last_corin_foothold1,sampling_time)
       
corin_foothold1_angular_velocity(:,ii) = angular_foothold1_velocity;
corin_foothold1_linear_velocity(:,ii) = linear_foothold1_velocity;


%% compute body velocity
current_base = vi.get_object_pose('/hexapod/body',-1,vi.OP_BLOCKING);
[vec_x_ref2base_dot_vrep,angular_w_matlab,linear_v_matlab] = compute_estimated_velocity(current_base,last_base,sampling_time);
[linear_v_vrep,angular_w_vrep] = vi.get_body_velocity_from_vrep('/hexapod/body');
if norm(angular_w_vrep)>0.3
    aaa = 0;
end

corin_angular_velocity_matlab(:,ii) = angular_w_matlab;
corin_angular_velocity_vrep(:,ii) = angular_w_vrep;
corin_linear_velocity_vrep(:,ii) = linear_v_vrep;
corin_linear_velocity_matlab(:,ii) = linear_v_matlab;

vec_x_ref2base_dot_vrep = vec8(vec_x_ref2base_dot_vrep);


%% compute the control input
    [corin_u,norm_function,vec_base_dot_matlab] = corin_controller.compute_setpoint_control_signal(corin_q,Xd,x_ref2f1_dot);
%     corin_u = corin_controller.compute_setpoint_control_signal(corin_q,Xd);
    matlab_jacobian = corin.pose_jacobian(corin_q);
    jacobian_abs = matlab_jacobian(1:8,1:11);
    vec_x_ref2f1_dot = vec8(x_ref2f1_dot);
    vec_abs = zeros(11,1);
    vec_abs(1:8) = vec_x_ref2f1_dot;
    leg1_q_dot = (current_leg1_q-last_leg1_q)/sampling_time;
    vec_abs(9:11) = leg1_q_dot;
    
    vec_x_ref2base_dot_matlab = jacobian_abs * vec_abs; 
    vec_error_x_ref2base_dot_matlab_and_vrep = vec_x_ref2base_dot_matlab-vec_x_ref2base_dot_vrep;
    norm_of_the_erroe_x_ref2_base_dot_matlab_and_vrep = norm(vec_error_x_ref2base_dot_matlab_and_vrep);
    
    [~,angular_velocity_jacobian,linear_velocity_jacobian]=compute_velocity_from_xdot(vec_x_ref2base_dot_matlab,current_base);
    corin_angular_velocity_jacobian(:,ii) = angular_velocity_jacobian;
    corin_linear_velocity_jacobian(:,ii) = linear_velocity_jacobian;
    
    
    if norm_of_the_erroe_x_ref2_base_dot_matlab_and_vrep > 1
        bbb=0;
    end
    
%     corin_q(9:26) = corin_q(9:26)+corin_u;
    


if a == 50
    b =0;
end

    
    
    %% Send desired values
%     corin_hexapod.send_target_joint_to_vrep(corin_q(9:26));
x_current = corin.fkm(corin_q);
x_current_abs = current_base;

each_loop_trans = 0*i_ + 0.01*j_ + 0*k_;
each_loop_rot = cos(pi/100) + j_*sin(pi/100);
each_loop_step_pose = each_loop_rot + E_*0.5*each_loop_trans*each_loop_rot;
each_loop_step_pose = normalize(each_loop_step_pose);

    x_ref_to_current_hexapod = vi.get_object_pose('/hexapod',-1,vi.OP_BLOCKING);
    body_to_hexapod = current_base' * x_ref_to_current_hexapod;
    
    vi.set_object_pose('/hexapod',x_current_abs * each_loop_step_pose * body_to_hexapod);
    corin_q(9:11) = corin_q(9:11) - 0.05;
    corin_hexapod.send_target_joint_to_vrep(corin_q(9:26));
    
    
    
    last_corin_foothold1 = current_corin_foothold1;
    last_base = current_base;
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
    
    %% Get data to plot them later
    corin_tracking_error_norm(:,ii) = norm(corin_controller.get_last_error_signal());
    corin_control_inputs(:,ii) = corin_u;
    corin_q_vector(:,ii) = corin_q;
    task_error = corin_controller.get_last_error_signal();
    corin_absolute_error_norm(:,ii) = norm(task_error(1:8));
    corin_relative_error_norm(:,ii) = norm(task_error(9:48));
    
    corin_base_cot_vrep(:,ii) = vec_x_ref2base_dot_vrep(1:8);
    corin_base_cot_matlab(:,ii) = vec_x_ref2base_dot_matlab(1:8);
    corin_norm_error_dot(:,ii) = norm_of_the_erroe_x_ref2_base_dot_matlab_and_vrep;
    
%     corin_velocity_error_norm(:,ii) = error_111;
    function_norm(:,ii) = norm_function;
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
T=0:sampling_time:total_time
figure(1)
plot(T,corin_tracking_error_norm)
title('norm of corin tracking error')
xlabel('Time/s') 
ylabel('corin tracking error norm') 
figure(2)
plot(T,corin_absolute_error_norm)
title('norm of corin absolute pose error')
xlabel('Time/s') 
ylabel('corin absolute pose error norm') 
figure(3)
plot(T,corin_relative_error_norm)
title('norm of corin relative pose error')
xlabel('Time/s') 
ylabel('corin relative pose error norm') 
figure(4)
plot(T,function_norm)
title('norm of cost function')
xlabel('Time/s') 
ylabel('cost function norm') 

figure(5)
plot(T,corin_angular_velocity_vrep(1,:))
hold on;
plot(T,corin_angular_velocity_vrep(2,:))
hold on;
plot(T,corin_angular_velocity_vrep(3,:))
hold on;
plot(T,corin_angular_velocity_matlab(1,:),':')
hold on;
plot(T,corin_angular_velocity_matlab(2,:),':')
hold on;
plot(T,corin_angular_velocity_matlab(3,:),':')
hold on;
title('velocity')
xlabel('Time/s') 
ylabel('velocity from vrep and matlab') 
legend('angular velocity from vrep rx','angular velocity from vrep ry','angular velocity from vrep rz',...
    'angular velocity from matlab rx','angular velocity from matlab ry','angular velocity from matlab rz')

figure(6)
plot(T,corin_linear_velocity_vrep(1,:))
hold on;
plot(T,corin_linear_velocity_vrep(2,:))
hold on;
plot(T,corin_linear_velocity_vrep(3,:))
hold on;
plot(T,corin_linear_velocity_matlab(1,:),':')
hold on;
plot(T,corin_linear_velocity_matlab(2,:),':')
hold on;
plot(T,corin_linear_velocity_matlab(3,:),':')
hold on;
title('velocity')
xlabel('Time/s') 
ylabel('velocity from vrep and matlab') 
legend('linear velocity from vrep x','linear velocity from vrep y','linear velocity from vrep z',...
    'linear velocity from matlab x','linear velocity from matlab y','linear velocity from matlab z')

figure(7);
plot(T,corin_base_cot_vrep(1,:))
hold on;
plot(T,corin_base_cot_matlab(1,:),':')
hold on;
title('first value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (1) from vrep and matlab') 
legend('base dot 1 vrep','base dot 1 matlab')

figure(8);
plot(T,corin_base_cot_vrep(2,:))
hold on;
plot(T,corin_base_cot_matlab(2,:),':')
hold on;
title('second value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (2) from vrep and matlab') 
legend('base dot 2 vrep','base dot 2 matlab')

figure(9);
plot(T,corin_base_cot_vrep(3,:))
hold on;
plot(T,corin_base_cot_matlab(3,:),':')
hold on;
title('third value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (3) from vrep and matlab') 
legend('base dot 3 vrep','base dot 3 matlab')

figure(10);
plot(T,corin_base_cot_vrep(4,:))
hold on;
plot(T,corin_base_cot_matlab(4,:),':')
hold on;
title('forth value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (4) from vrep and matlab') 
legend('base dot 4 vrep','base dot 4 matlab')

figure(11);
plot(T,corin_base_cot_vrep(5,:))
hold on;
plot(T,corin_base_cot_matlab(5,:),':')
hold on;
title('fifth value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (5) from vrep and matlab') 
legend('base dot 5 vrep','base dot 5 matlab')

figure(12);
plot(T,corin_base_cot_vrep(6,:))
hold on;
plot(T,corin_base_cot_matlab(6,:),':')
hold on;
title('sixth value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (6) from vrep and matlab') 
legend('base dot 6 vrep','base dot 6 matlab')

figure(13);
plot(T,corin_base_cot_vrep(7,:))
hold on;
plot(T,corin_base_cot_matlab(7,:),':')
hold on;
title('seventh value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (7) from vrep and matlab') 
legend('base dot 7 vrep','base dot 7 matlab')

figure(14);
plot(T,corin_base_cot_vrep(8,:))
hold on;
plot(T,corin_base_cot_matlab(8,:),':')
hold on;
title('eighth value on the vector base dot')
xlabel('Time/s') 
ylabel('base dot (8) from vrep and matlab') 
legend('base dot 8 vrep','base dot 8 matlab')

figure(15);
plot(T,corin_norm_error_dot)
title('norm of the error for xref2basedot on matlab and vrep')
xlabel('Time/s') 

figure(16)
plot(T,corin_linear_velocity_jacobian(1,:))
hold on;
plot(T,corin_linear_velocity_jacobian(2,:))
hold on;
plot(T,corin_linear_velocity_jacobian(3,:))
hold on;
plot(T,corin_linear_velocity_matlab(1,:),':')
hold on;
plot(T,corin_linear_velocity_matlab(2,:),':')
hold on;
plot(T,corin_linear_velocity_matlab(3,:),':')
hold on;
title('base linear velocity')
xlabel('Time/s') 
ylabel('base linear velocity') 
legend('linear velocity from jacobian x','linear velocity from jacobian y','linear velocity from jacobian z',...
    'linear velocity from matlab x','linear velocity from matlab y','linear velocity from matlab z')

figure(17)
plot(T,corin_angular_velocity_jacobian(1,:))
hold on;
plot(T,corin_angular_velocity_jacobian(2,:))
hold on;
plot(T,corin_angular_velocity_jacobian(3,:))
hold on;
plot(T,corin_angular_velocity_matlab(1,:),':')
hold on;
plot(T,corin_angular_velocity_matlab(2,:),':')
hold on;
plot(T,corin_angular_velocity_matlab(3,:),':')
hold on;
title('base angular velocity')
xlabel('Time/s') 
ylabel('angular velocity') 
legend('angular velocity from jacobian rx','angular velocity from jacobian ry','angular velocity from jacobian rz',...
    'angular velocity from matlab rx','angular velocity from matlab ry','angular velocity from matlab rz')

figure(18)
plot(T,corin_foothold1_angular_velocity(1,:))
hold on;
plot(T,corin_foothold1_angular_velocity(2,:))
hold on;
plot(T,corin_foothold1_angular_velocity(3,:))
hold on;
plot(T,corin_angular_velocity_matlab(1,:),':')
hold on;
plot(T,corin_angular_velocity_matlab(2,:),':')
hold on;
plot(T,corin_angular_velocity_matlab(3,:),':')
hold on;
title('angular velocity')
xlabel('Time/s') 
ylabel('angular velocity of foothold1') 
legend('angular velocity foothold1 rx','angular velocity foothold1 ry','angular velocity foothold1 rz',...
    'angular velocity base rx','angular velocity base ry','angular velocity base rz');

figure(19)
plot(T,corin_foothold1_linear_velocity(1,:))
hold on;
plot(T,corin_foothold1_linear_velocity(2,:))
hold on;
plot(T,corin_foothold1_linear_velocity(3,:))
hold on;
title('linear velocity')
xlabel('Time/s') 
ylabel('linear velocity of foothold1') 
legend('linear velocity foothold1 x','linear velocity foothold1 y','linear velocity foothold1 z');
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

% compute the time-varying reference for the Kuka LWR 4 end-effector
function [x, xdot] = compute_lwr4_reference(lwr4, simulation_parameters, x0, t)
include_namespace_dq;

% parameters for the trajectory
dispz = simulation_parameters.dispz;
wd = simulation_parameters.wd;
wn = simulation_parameters.wn;

% calculate the time-varing plane orientation with respect to the robot
% BASE.
phi = (pi/2)*sin(wn*t);
r = cos(phi/2) + k_ * sin(phi/2);

z = dispz*cos(wd*t)*k_;
p = 1 + E_ * 0.5 * z;

x = r*x0*p;

% calculate its time derivative
phidot = (pi/2)*cos(wn*t)*wn;
rdot = 0.5*(-sin(phi/2) + k_*cos(phi/2))*phidot;
pdot = -E_*0.5*dispz*wd*sin(wd*t)*k_;

xdot = rdot*x0*p + r*x0*pdot;

% The trajectory,including the feedforward term, has been calculated with
% respect to the manipulator base. Therefore, we need to calculate them
% with respect to the global reference frame.
x = lwr4.get_reference_frame()*x;
xdot = lwr4.get_reference_frame()*xdot;
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

function [x_dot,angular_velocity,linear_velocity] = compute_estimated_velocity(current_pose,last_pose,sampling_time)

x_trans = last_pose'* current_pose;
unit_dq = DQ(1);
norm1 = norm(x_trans-unit_dq);
norm2 = norm(x_trans+unit_dq);
value1_vec = vec8(norm1);
value1 = value1_vec(1);
value2_vec=vec8(norm2);
value2 = value2_vec(1);
if value1 <= value2
    xi = 2*log(x_trans)/ sampling_time;
else
    xi = 2*log(-x_trans)/ sampling_time;
end

xi = Ad(last_pose,xi);
angular_velocity_DQ = xi.P;
angular_velocity = vec3(angular_velocity_DQ);

if norm(angular_velocity)>0.1
    aaa = 0
end
p = last_pose.translation;
linear_velocity_DQ = xi.D-cross(p,angular_velocity);
linear_velocity = vec3(linear_velocity_DQ);
a = norm(angular_velocity);
if a >4
    b = 0
end

x_dot =0.5* xi * current_pose;

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
