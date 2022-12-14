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

function vrep_hexapodscene_dqcontroller(simulation_parameters)

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

%% Get robot joint information from V-REP
% corin_reference_frames = {'/footTarget0','footTip1','joint13_',...
%     'lm_q1_joint','lm_q2_joint','lm_q3_joint',...
%     'lr_q1_joint','lr_q2_joint','lr_q3_joint',...
%     'rf_q1_joint','rf_q2_joint','rf_q3_joint',...
%     'rm_q1_joint','rm_q2_joint','rm_q3_joint',...
%     'rr_q1_joint','rr_q2_joint','rr_q3_joint'};

%% Load DQ Robotics kinematics
corin  = corin_hexapod.kinematics();
% youbot = youbot_vreprobot.kinematics();

%% Initialize controllers
% lwr4_controller = DQ_PseudoinverseController(corin);
% lwr4_controller.set_control_objective(ControlObjective.Pose);
% lwr4_controller.set_gain(10);

solver = DQ_QuadprogSolver;
corin_controller = DQ_ClassicQPController_Hexapod(corin,solver);
corin_controller.set_control_objective(ControlObjective.HexapodTask);
corin_controller.set_gain(0.01);
corin_controller.set_damping(0.01);

sampling_time = 0.05; % V-REP's sampling time is 50 ms.
total_time = simulation_parameters.total_time; % Total time, in seconds.

%% Set the initial robots configurations
% lwr4_q  = [0; 1.7453e-01; 0; 1.5708e+00; 0; 2.6273e-01; 0];
corin_base = get_base_from_vrep(vi,'/hexapod/body');
corin_q = corin_hexapod.get_q_from_vrep();
% corin_q = [current_corin_base;  0;0;0;    0;0;0;    0;0;0;     0;0;0;    0;0;0; 0;0;0];

% corin_hexapod.send_q_to_vrep(corin_q);

% Get current end-effector pose with respect to its own base

% corin.plot(corin_q);

% **** corin_x0  = corin.get_reference_frame()'*corin.fkm(corin_q);


%% Initalize arrays to store all signals resulting from the simulation
max_iterations = round(total_time/sampling_time);

corin_tracking_error_norm = zeros(1,max_iterations);
corin_control_inputs = zeros(corin.get_dim_configuration_space(),...
    max_iterations);
corin_q_vector = zeros(26, ...
    max_iterations);

% index of the auxiliary arrays used to plot data
ii = 1;
%% Set reference for the manipulator and the mobile manipulator
include_namespace_dq

foothold1 = corin_hexapod.get_reference_joint1_from_vrep;
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
r =cos(pi/12) + k_*sin(pi/12);
p = -0.275*i_ + 0.325*j_ + 0.088*k_;
desired_base_pose = r + E_*0.5*p*r;
corin_xd(1) = desired_base_pose;

Xd= [];
for i = 1:6
    x = vec8(corin_xd(i));
    Xd = [Xd;x];
end


% % % first calculate the constraints
base_velocity = compute_estimated_velocity(corin_base,corin_base,sampling_time);
% vec_base = DQ(0);
[Constraint_matrix,Constraint_Vector] = corin.get_constraint_matrix_and_vector(corin_q,base_velocity);
Constraint_Vector = -Constraint_Vector;
% control.set_inequality_constraint(Constraint_matrix,Constraint_Vector);
corin_controller.set_equality_constraint(Constraint_matrix,Constraint_Vector);
a = 0;

current_corin_base = corin_base;
last_corin_base = corin_base;
current_footTip0 = 0;
last_footTip0 = vi.get_object_pose('/hexapod/footTip0');

%% Control loop
for t=0:sampling_time:total_time
%    pause(0.05);
    %% Compute control signal for the youbot
    a = a+1;
%     foothold1 = corin_hexapod.get_reference_joint1_from_vrep;
%     corin.set_reference_to_first_feethold_vrep(foothold1);
%% set constraint
current_corin_base = get_base_from_vrep(vi,'/hexapod/body');
current_footTip0 = vi.get_object_pose('/hexapod/footTip0');
footTip0_velocity = compute_estimated_velocity(current_footTip0,last_footTip0,sampling_time);
base_velocity = compute_estimated_velocity(current_corin_base,last_corin_base,sampling_time);
    [Constraint_matrix,Constraint_Vector] = corin.get_constraint_matrix_and_vector(corin_q,base_velocity);

    % control.set_inequality_constraint(Constraint_matrix,Constraint_Vector);
    corin_controller.set_equality_constraint(Constraint_matrix,Constraint_Vector);
%% compute the control input
    corin_u = corin_controller.compute_setpoint_control_signal(corin_q,Xd,footTip0_velocity);
    corin_q(9:26) = corin_q(9:26)+corin_u;
    last_corin_base =  current_corin_base;
    last_footTip0 = current_footTip0;

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

function estimated_velocity = compute_estimated_velocity(current_base,last_base,sampling_time)
x_trans = last_base'* current_base;
estimated_velocity = (log(x_trans) * current_base) / sampling_time;
end