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

function vrep_hexapodscene_dqcontroller_with_constraint_2(simulation_parameters)

include_namespace_dq;

%% Simulation convenience flags
SHOW_FRAMES = simulation_parameters.show_frames;

%% Initialize V-REP interface
vi = DQ_VrepInterface;
vi.disconnect_all(); % For testing, can be removed for release
vi.connect('127.0.0.1',19997);
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

sampling_time = 0.001; % V-REP's sampling time is 50 ms.
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

corin_angular_velocity_vrep = zeros(3,max_iterations);
corin_angular_velocity_matlab = zeros(3,max_iterations);




% index of the auxiliary arrays used to plot data
ii = 1;
%% Set reference for the manipulator and the mobile manipulator
include_namespace_dq

foothold1 = corin_hexapod.get_reference_feethold1_from_vrep;
% corin.set_reference_to_first_feethold(corin_q);
corin.set_reference_to_first_feethold_vrep(foothold1);



% reference_corin_q = [corin_q(1:8); 0.3;0.3;0.3;     0.3;0.3;0.3;     0.3;0.3;0.3;      0.3;0.3;0.3;     0.3;0.3;0.3;  0.3;0.3;0.3;];
% x_ref = corin.fkm(reference_corin_q);
x_origin = corin.fkm(corin_q);
% corin_xd = x_ref;
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

v = vi.get_body_velocity_from_vrep('/hexapod');



a = 0;
current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0');
last_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0');
current_base = vi.get_object_pose('/hexapod/body');
last_base = vi.get_object_pose('/hexapod/body');

%% Control loop
for t=0:sampling_time:total_time
%    pause(0.05);
[linear_v,angular_w_vrep] = vi.get_body_velocity_from_vrep('/hexapod/body');
x_current = corin.fkm(corin_q);
for i = 2:6
    x = vec8(x_current(i));
    Xd(8*(i-1)+1:8*i)=x;
end
    %% Compute control signal for the youbot
    a = a+1;
    pause(0.05);
    foothold1 = vi.get_object_pose('/hexapod/footTip0');
    corin.set_reference_to_first_feethold_vrep(foothold1);
    
    corin_q = corin_hexapod.get_q_from_vrep();
    
    joint1_pose = corin_hexapod.get_object_pose_from_vrep_world1();
    footTip0 = corin_hexapod.get_foothold1_pose_from_vrep();
    res = joint1_pose'*footTip0;
%% get foothold1 velocity
% current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0');
% current_corin_foothold2 = vi.get_object_pose('/hexapod/footTip0');
% % get_base_from_vrep(vi,'/hexapod/footTip0');
% % current_footTip0 = vi.get_object_pose('/hexapod/footTip0');
% current_base_trans = current_corin_foothold1.translation;
% current_base_rot = current_corin_foothold1.rotation;
% 
% foothold1_velocity = compute_estimated_velocity(current_corin_foothold1,last_corin_foothold1,sampling_time);
Down = vi.get_object_pose('/Down');
Up = vi.get_object_pose('/Up');
Left = vi.get_object_pose('/Left');
Right = vi.get_object_pose('/Right');



%% set constraint
current_base = vi.get_object_pose('/hexapod/body');
fkm_res = corin.fkm(corin_q);
current_base1 = fkm_res(1);
[base_velocity,base_angular_velocity] = compute_estimated_velocity(current_base,last_base,sampling_time);
[Constraint_matrix,Constraint_Vector] = corin.get_constraint_matrix_and_vector(corin_q,base_velocity);
%      corin_controller.set_inequality_constraint(Constraint_matrix,Constraint_Vector);
corin_controller.set_equality_constraint(Constraint_matrix,Constraint_Vector);

% Joint_velocity_constraint_matrix = eye(18);
% joint_velocity_max = 2 * pi * sampling_time;
% % joint_velocity_max = 1;
% Joint_velocity_constraint_vector = zeros(18,1);
% Joint_velocity_constraint_vector(:)=joint_velocity_max;
% corin_controller.set_inequality_constraint(Joint_velocity_constraint_matrix,Joint_velocity_constraint_vector);


%% compute the control input
    corin_u = corin_controller.compute_setpoint_control_signal(corin_q,Xd,foothold1_velocity)
%     corin_u = corin_controller.compute_setpoint_control_signal(corin_q,Xd);
    corin_q(9:26) = corin_q(9:26)+corin_u;
    last_corin_foothold1 = current_corin_foothold1;
    last_base = current_base;


if a == 50
    b =0;
end

    
    
    %% Send desired values
    corin_hexapod.send_target_joint_to_vrep(corin_q(9:26));
    
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

    corin_angular_velocity_vrep(:,ii) = angular_w_vrep;
    corin_angular_velocity_matlab(:,ii) = base_angular_velocity;
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

function [estimated_velocity, angular_velocity] = compute_estimated_velocity(current_pose,last_pose,sampling_time)
x_trans = last_pose'* current_pose;
% % x_trans = current_pose * last_pose';
% estimated_velocity = ((log(x_trans)/ sampling_time) * current_pose);

xi = 2*log(x_trans)/ sampling_time;
angular_velocity_DQ = xi.P;
angular_velocity = vec3(angular_velocity_DQ);
estimated_velocity =0.5* xi * current_pose;

% x_trans =  last_pose' * current_pose;
% kxi_1 = 2*log(x_trans)/ sampling_time;
% % kxi_2 = 2 * log(last_pose) /sampling_time
% % kxi = kxi_2 + Ad(last_pose,kxi_1);
% kxi = Ad(last_pose,kxi_1);
% estimated_velocity =0.5 * kxi * current_pose;
end
