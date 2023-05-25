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

function vrep_scene_dqcontroller_demo(simulation_parameters)

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
youbot_vreprobot = YouBotVrepRobot('youBot',vi);
lwr4_vreprobot = LBR4pVrepRobot('LBR4p',vi);
%% Get robot joint information from V-REP
youbot_reference_frames = {'YoubotArm0','YoubotArm1','YoubotArm2','YoubotArm3','YoubotArm4'};

%% Load DQ Robotics kinematics
youbot = youbot_vreprobot.kinematics();

%% Initialize controllers

solver = DQ_QuadprogSolver;
youbot_controller = DQ_ClassicQPController(youbot,solver);
youbot_controller.set_control_objective(ControlObjective.Pose);
youbot_controller.set_gain(10);
youbot_controller.set_damping(0.01);

% lwr4_controller = DQ_PseudoinverseController(lwr4);
% lwr4_controller.set_control_objective(ControlObjective.Pose);
% lwr4_controller.set_gain(10);

sampling_time = 0.01; % V-REP's sampling time is 50 ms.
total_time = simulation_parameters.total_time; % Total time, in seconds.

%% Set the initial robots configurations
% Get current end-effector pose with respect to its own base
LBR4p_q = lwr4_vreprobot.get_q_from_vrep();

%% Initalize arrays to store all signals resulting from the simulation
max_iterations = round(total_time/sampling_time);
youbot_tracking_error_norm = zeros(1,max_iterations);
youbot_control_inputs = zeros(youbot.get_dim_configuration_space(),...
    max_iterations);
youbot_q_vector = zeros(youbot.get_dim_configuration_space(), ...
    max_iterations);
corin_angular_velocity_vrep = zeros(3,max_iterations);
corin_angular_velocity_matlab = zeros(3,max_iterations);

% index of the auxiliary arrays used to plot data
ii = 1;

% '/LBR4p/LBR4p_joint7'
% current_pose =  vi.get_object_pose('/LBR4p/LBR4p_joint6');
% last_pose = vi.get_object_pose('/LBR4p/LBR4p_joint6');
% '/youBot'

current_pose =  vi.get_object_pose('/youBot/');
last_pose = vi.get_object_pose('/youBot/');


%% Control loop
for t=0:sampling_time:total_time
    last_pose = current_pose;
    vi.synchronous_trigger();
    pause(0.1)
    youBot_q = youbot_vreprobot.get_q_from_vrep_wheel();
    current_pose =  vi.get_object_pose( '/youBot/',-1,vi.OP_BLOCKING);
    [~, base_angular_velocity] = compute_estimated_velocity(current_pose,last_pose,sampling_time);
    [~,angular_w_vrep] = vi.get_body_velocity_from_vrep( '/youBot/');
        corin_angular_velocity_vrep(:,ii) = angular_w_vrep;
    corin_angular_velocity_matlab(:,ii) = base_angular_velocity;
    ii = ii+1;
   

    youBot_u = [10,-10,-10,10];
    youBot_q = youBot_q+youBot_u'*sampling_time;
    youbot_vreprobot.send_q_to_vrep_wheel(youBot_q);
    
%     youBot_u = youbot_vreprobot.inverse_kinematic_base(0,0,0.3);
%     LBR4p_u = [0.1,0.1,0.1,0.1,0.1,0.1,0.1];
%     LBR4p_q = LBR4p_q + LBR4p_u';
%     lwr4_vreprobot.send_target_joint_to_vrep(LBR4p_q);  
end
T = 0:sampling_time:total_time;
% T = 0:sampling_time:total_time;
figure(1)
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

function [estimated_velocity, angular_velocity] = compute_estimated_velocity(current_pose,last_pose,sampling_time)
x_trans =  last_pose' * current_pose;

kxi = 2*log(x_trans)/ sampling_time;
% kxi_2 = 2 * log(last_pose) /sampling_time
% kxi = kxi_2 + Ad(last_pose,kxi_1);

% kxi = Ad(last_pose,kxi);
angular_velocity_DQ = kxi.P;
angular_velocity = vec3(angular_velocity_DQ);
estimated_velocity =0.5 * kxi * current_pose;
if norm(angular_velocity) > 100
    a = 0
end
end