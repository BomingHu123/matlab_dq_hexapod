% Example used in the paper Adorno and Marinho. 2020. 鈥淒Q Robotics: A
% Library for Robot Modeling and Control.鈥? IEEE Robotics & Automation
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

function vrep_hexapodscene_dqcontroller_with_constraint(simulation_parameters)

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
corin_hexapod = DQVrepHexapod('hexapod',vi)
corin_hexapod_new = DQVrepHexapod('hexapod_new',vi)
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
corin_controller.set_gain(5);
corin_controller.set_damping(0.01);

sampling_time = 0.05; % V-REP's sampling time is 50 ms.
total_time = simulation_parameters.total_time; % Total time, in seconds.

%% Set the initial robots configurations
corin_base = vi.get_object_pose('/hexapod/body');
corin_q = corin_hexapod.get_q_from_vrep();
% corin_q = [current_corin_base;  0;0;0;    0;0;0;    0;0;0;     0;0;0;    0;0;0; 0;0;0];


%% Initalize arrays to store all signals resulting from the simulation
max_iterations = round(total_time/sampling_time);

corin_whole_tracking_error_norm = zeros(1,max_iterations);
corin_tracking_error_norm = zeros(1,max_iterations);
corin_jacobian_error_norm = zeros(1,max_iterations);

corin_control_inputs = zeros(corin.get_dim_configuration_space(),...
    max_iterations);
corin_q_vector = zeros(26, ...
    max_iterations);

corin_absolute_error_norm = zeros(1,max_iterations);
corin_relative_error_norm = zeros(1,max_iterations);
corin_relative1_error_norm = zeros(1,max_iterations);
corin_relative2_error_norm = zeros(1,max_iterations);
corin_relative3_error_norm = zeros(1,max_iterations);
corin_relative4_error_norm = zeros(1,max_iterations);
corin_relative5_error_norm = zeros(1,max_iterations);

norm_of_u_leg = zeros(1,max_iterations);
norm_of_u_leg1 = zeros(1,max_iterations);
norm_of_u_leg2 = zeros(1,max_iterations);
norm_of_u_leg3 = zeros(1,max_iterations);
norm_of_u_leg4 = zeros(1,max_iterations);
norm_of_u_leg5 = zeros(1,max_iterations);
norm_of_u_leg6 = zeros(1,max_iterations);

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

rank_jacobian = zeros(1,max_iterations);

last_q_dot_vec = zeros(18,max_iterations);
corin_q_vec=zeros(18,max_iterations);
corin_q_plus_u = zeros(18,max_iterations);


% index of the auxiliary arrays used to plot data
ii = 1;
%% Set reference for the manipulator and the mobile manipulator
include_namespace_dq

foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
% corin.set_reference_to_first_feethold(corin_q);
corin.set_reference_to_reference_foothold_vrep(foothold1);
corin.set_reference_foothold_number(2)

%---------使用fkm计算一个xd----------------------------------------
% corin_q_task = [corin_q(1:8); -0.4107; -0.2185; 2.1084; -0.1685; -0.4499; 2.1154; -0.2292; -0.2193;  1.6489; -0.4337;  0.1786;  1.1435; -0.6242; 0.2880;  1.1576; -0.6619; 0.0817; 1.6627];

%-----rotate arount x axis--------------------------------------
% a = [-0.1789, -5.035,+9.953e+01, -9.979e-01, +2.662e+01, +6.637e+01, +1.052e+00, +2.663e+01, +6.618e+01, +4.345e-01, -5.181e+00, +9.948e+01,-1.077e+00,-3.881e+01,+1.277e+02,+1.055e+00,-3.854e+01,+1.276e+02]/57.3;
% corin_q_task = [corin_q(1:8); a'];
% % vec_ref_to_f1 = [0.97902, - 0.043705, 0.18231, - 0.079771,- 0.023776, 0.22033, 0.20339, 0.05231];
% vec_ref_to_f1 =[0.67728, + 0.096682, + 0.72665, + 0.062576, - 0.12101, - 0.011339, + 0.11774, - 0.039992];
% ref_to_f1 = DQ(vec_ref_to_f1);
% ref_to_f1 = normalize(ref_to_f1);
% 
% % vec_f1 = [0.54108, 0.15881, 0.80685, - 0.17609, - 0.10735, - 0.055752, 0.068152, - 0.067872];
% % f1 = DQ(vec_f1);
% % f1 = normalize(f1);
% 
% corin.set_reference_to_reference_foothold_vrep(ref_to_f1);
% x_origin = corin.fkm(corin_q_task);
% corin_xd = x_origin;
% 
% r =cos(0) + j_*sin(0);
% p = -0.275*i_ + 0.325*j_ + 0.088*k_;
% 
% desired_base_pose = r + E_*0.5*p*r;
% corin_xd(1) = desired_base_pose;

%--------rotate arount y axis --------------------------------
% a = [0.0013;0.0772; 1.6469;-0.0028; -0.1493;1.8034;0.0057;-0.5596;2.0413;0.0013;-0.8302;2.2462;-0.0022; -0.5520; 2.0417; 0.0059; -0.1415;1.8036];
% corin_q_task = [corin_q(1:8); a];
% x_origin = corin.fkm(corin_q_task);
% r =cos(-pi/10) + j_*sin(-pi/10);
% p = -0.275*i_ + 0.325*j_ + 0.088*k_;
% desired_base_pose = r + E_*0.5*p*r;
% x_origin(1) = desired_base_pose;
% corin_xd = x_origin;

%----rotate around z axis and lift foot and lift body----------
% a = [0.9044,   -0.1212,    1.6703,    0.9109,   -0.4802,    0.4969,    0.9024,   -0.3257,    1.9015,    0.8999,...
%     -0.1213,    1.6839,    0.8970,   -0.0773,    1.6492,    0.9029,   -0.0908,    1.6694];
% corin_q_task = [corin_q(1:8); a'];
% x_origin = corin.fkm(corin_q_task);
% r =cos(-pi/10) + k_*sin(-pi/10);
% p = -0.275*i_ + 0.325*j_ + 0.15*k_;
% desired_base_pose = r + E_*0.5*p*r;
% x_origin(1) = desired_base_pose;
% corin_xd = x_origin;

%-------------对比的reference---------------------------------
% corin_q_test = corin_hexapod.get_q_from_vrep();
% corin.set_reference_to_reference_foothold_vrep(foothold1);
% xd_test = corin.fkm(corin_q_test);

%---------使用正常的xd---------------------------------------
x_origin = corin.fkm(corin_q);
corin_xd = x_origin;
% 
r =cos(0) + j_*sin(0);
p = -0.275*i_ + 0.325*j_ + 0.15*k_;

desired_base_pose = r + E_*0.5*p*r;
corin_xd(1) = desired_base_pose;
% 
% 
% 
% vec_f1_to_base = [0.96672, -0.030965, - 0.15446, 0.20156, - 0.010269, - 0.092852, + 0.0033054, 0.037521];
% f1_to_base = DQ(vec_f1_to_base);
% f1_to_base = normalize(f1_to_base);


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



last_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
last_foothold2 = vi.get_object_pose('/hexapod/footTip1',-1,vi.OP_BLOCKING);
last_foothold3 = vi.get_object_pose('/hexapod/footTip2',-1,vi.OP_BLOCKING);
last_foothold4 = vi.get_object_pose('/hexapod/footTip3',-1,vi.OP_BLOCKING);
last_foothold5 = vi.get_object_pose('/hexapod/footTip4',-1,vi.OP_BLOCKING);
last_foothold6 = vi.get_object_pose('/hexapod/footTip5',-1,vi.OP_BLOCKING);

foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
corin.set_reference_to_reference_foothold_vrep(foothold1);

last_leg1_q = corin_q(9:11);
last_leg_q = corin_q(9:26);
%% Control loop
for t=0:sampling_time:total_time
    %    pause(0.05);
    vi.synchronous_trigger();
    %% Compute control signal for the youbot
    a = a+1;
    %     pause(0.05);
    foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
    corin.set_reference_to_reference_foothold_vrep(foothold1);
    if ii == 55
        a = 0;
    end
    %     foothold2 = vi.get_object_pose('/hexapod/footTip1',-1,vi.OP_BLOCKING);
    %     foothold3 = vi.get_object_pose('/hexapod/footTip2',-1,vi.OP_BLOCKING);
    %     foothold4 = vi.get_object_pose('/hexapod/footTip3',-1,vi.OP_BLOCKING);
    %     foothold5 = vi.get_object_pose('/hexapod/footTip4',-1,vi.OP_BLOCKING);
    %     foothold6 = vi.get_object_pose('/hexapod/footTip5',-1,vi.OP_BLOCKING);
    %     vecfoothold1trans = vec3(foothold1.translation);
    %     vecfoothold2trans = vec3(foothold2.translation);
    %     vecfoothold3trans = vec3(foothold3.translation);
    %     vecfoothold4trans = vec3(foothold4.translation);
    %     vecfoothold5trans = vec3(foothold5.translation);
    %     vecfoothold6trans = vec3(foothold6.translation);
    %
    %     corin_foothold1_pose_vrep(:,ii) = vecfoothold1trans;
    %     corin_foothold2_pose_vrep(:,ii) = vecfoothold2trans;
    %     corin_foothold3_pose_vrep(:,ii) = vecfoothold3trans;
    %     corin_foothold4_pose_vrep(:,ii) = vecfoothold4trans;
    %     corin_foothold5_pose_vrep(:,ii) = vecfoothold5trans;
    %     corin_foothold6_pose_vrep(:,ii) = vecfoothold6trans;
    
%         corin.set_reference_to_reference_foothold_vrep(foothold1);
    
    corin_q_vrep = corin_hexapod.get_q_from_vrep();
    current_leg1_q = corin_q(9:11);
    current_leg_q = corin_q_vrep(9:26);
    
    %% get foothold1 velocity
    current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
    [x_ref2f1_dot,~,~] = compute_estimated_velocity(current_corin_foothold1,last_corin_foothold1,sampling_time);
    
    %% set constraint
    current_base = vi.get_object_pose('/hexapod/body/',-1,vi.OP_BLOCKING);
    fkm_res = corin.fkm(corin_q);
    [x_ref2base_dot_approximation,angular_w_matlab,linear_v_matlab] = compute_estimated_velocity(current_base,last_base,sampling_time);
    [Constraint_matrix,Constraint_Vector,J_whole] = corin.get_constraint_matrix_and_vector(corin_q,x_ref2base_dot_approximation);
%          corin_controller.set_equality_constraint(Constraint_matrix,Constraint_Vector);
    vec_max_foothold_linear_velocity = zeros(18,1);
    for j = 1:length(vec_max_foothold_linear_velocity)
        vec_max_foothold_linear_velocity(j) = 0.005;
    end
    Constraint_matrix_new = [Constraint_matrix;-Constraint_matrix];
    Constraint_Vector_new = [Constraint_Vector+vec_max_foothold_linear_velocity;-Constraint_Vector+vec_max_foothold_linear_velocity];
    corin_controller.set_inequality_constraint(Constraint_matrix_new,Constraint_Vector_new);
    leg_q_dot = (current_leg_q - last_leg_q)/sampling_time;
    
    last_q_dot_vec(:,ii)=leg_q_dot;
    
    if Constraint_matrix * leg_q_dot <  Constraint_Vector
        
        aaa = 0
        
    end
    
    vec_x_ref2base_dot_approximation = vec8(x_ref2base_dot_approximation);
    % Joint_velocity_constraint_matrix = eye(18);
    % joint_velocity_max = 2 * pi * sampling_time;
    % % joint_velocity_max = 1;
    % Joint_velocity_constraint_vector = zeros(18,1);
    % Joint_velocity_constraint_vector(:)=joint_velocity_max;
    % corin_controller.set_inequality_constraint(Joint_velocity_constraint_matrix,Joint_velocity_constraint_vector);
    
    vec_constraint = zeros(26,1);
    vec_constraint(1:8) = vec8(x_ref2base_dot_approximation);
    
    vec_constraint(9:26) = leg_q_dot;
    foothold_trans_dot = 2 * J_whole * vec_constraint;
    linear_velocity_foothold1_jacobian(:,ii) = foothold_trans_dot(1:3);
    linear_velocity_foothold2_jacobian(:,ii) = foothold_trans_dot(4:6);
    linear_velocity_foothold3_jacobian(:,ii) = foothold_trans_dot(7:9);
    linear_velocity_foothold4_jacobian(:,ii) = foothold_trans_dot(10:12);
    linear_velocity_foothold5_jacobian(:,ii) = foothold_trans_dot(13:15);
    linear_velocity_foothold6_jacobian(:,ii) = foothold_trans_dot(16:18);
    
    %     foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
    %     foothold2 = vi.get_object_pose('/hexapod/footTip1',-1,vi.OP_BLOCKING);
    %     foothold3 = vi.get_object_pose('/hexapod/footTip2',-1,vi.OP_BLOCKING);
    %     foothold4 = vi.get_object_pose('/hexapod/footTip3',-1,vi.OP_BLOCKING);
    %     foothold5 = vi.get_object_pose('/hexapod/footTip4',-1,vi.OP_BLOCKING);
    %     foothold6 = vi.get_object_pose('/hexapod/footTip5',-1,vi.OP_BLOCKING);
    %
    %     [~,~,linear_foothold1_right] = compute_estimated_velocity(foothold1,last_foothold1,sampling_time);
    %     [~,~,linear_foothold2_right] = compute_estimated_velocity(foothold2,last_foothold2,sampling_time);
    %     [~,~,linear_foothold3_right] = compute_estimated_velocity(foothold3,last_foothold3,sampling_time);
    %     [~,~,linear_foothold4_right] = compute_estimated_velocity(foothold4,last_foothold4,sampling_time);
    %     [~,~,linear_foothold5_right] = compute_estimated_velocity(foothold5,last_foothold5,sampling_time);
    %     [~,~,linear_foothold6_right] = compute_estimated_velocity(foothold6,last_foothold6,sampling_time);
    %
    %     linear_velocity_foothold1_right(:,ii) = linear_foothold1_right;
    %     linear_velocity_foothold2_right(:,ii) = linear_foothold2_right;
    %     linear_velocity_foothold3_right(:,ii) = linear_foothold3_right;
    %     linear_velocity_foothold4_right(:,ii) = linear_foothold4_right;
    %     linear_velocity_foothold5_right(:,ii) = linear_foothold5_right;
    %     linear_velocity_foothold6_right(:,ii) = linear_foothold6_right;
    
    
    %% compute body velocity
    current_base = vi.get_object_pose('/hexapod/body/',-1,vi.OP_BLOCKING);
    [~,angular_w_matlab,linear_v_matlab] = compute_estimated_velocity(current_base,last_base,sampling_time);
    [linear_base_v_vrep,angular_base_w_vrep] = vi.get_body_velocity_from_vrep('/hexapod/body');
    corin_base_angular_velocity_matlab(:,ii) = angular_w_matlab;
    corin_base_angular_velocity_vrep(:,ii) = angular_base_w_vrep;
    corin_base_linear_velocity_vrep(:,ii) = linear_base_v_vrep;
    corin_base_linear_velocity_matlab(:,ii) = linear_v_matlab;
    
    [linear_footTip1_v_vrep,~] = vi.get_body_velocity_from_vrep('/hexapod/footTip0');
    [linear_footTip2_v_vrep,~] = vi.get_body_velocity_from_vrep('/hexapod/footTip1');
    [linear_footTip3_v_vrep,~] = vi.get_body_velocity_from_vrep('/hexapod/footTip2');
    [linear_footTip4_v_vrep,~] = vi.get_body_velocity_from_vrep('/hexapod/footTip3');
    [linear_footTip5_v_vrep,~] = vi.get_body_velocity_from_vrep('/hexapod/footTip4');
    [linear_footTip6_v_vrep,~] = vi.get_body_velocity_from_vrep('/hexapod/footTip5');
    linear_velocity_foothold1_vrep(:,ii) = linear_footTip1_v_vrep;
    linear_velocity_foothold2_vrep(:,ii) = linear_footTip2_v_vrep;
    linear_velocity_foothold3_vrep(:,ii) = linear_footTip3_v_vrep;
    linear_velocity_foothold4_vrep(:,ii) = linear_footTip4_v_vrep;
    linear_velocity_foothold5_vrep(:,ii) = linear_footTip5_v_vrep;
    linear_velocity_foothold6_vrep(:,ii) = linear_footTip6_v_vrep;
    
    
    %% compute jacobian
    matlab_jacobian = corin.pose_jacobian(corin_q);
    rank_jacobian(:,ii) = rank(matlab_jacobian);
    
    jacobian_abs = matlab_jacobian(1:8,1:11);
    vec_x_ref2f1_dot = vec8(x_ref2f1_dot);
    vec_abs = zeros(11,1);
    vec_abs(1:8) = vec_x_ref2f1_dot;
    leg1_q_dot = (current_leg1_q-last_leg1_q)/sampling_time;
    vec_abs(9:11) = leg1_q_dot;
    
    vec_x_ref2base_dot_matlab = jacobian_abs * vec_abs;
    vec_error_x_ref2base_dot_matlab_and_vrep = vec_x_ref2base_dot_matlab-vec_x_ref2base_dot_approximation;
    norm_of_the_erroe_x_ref2_base_dot_matlab_and_vrep(:,ii) = norm(vec_error_x_ref2base_dot_matlab_and_vrep);
    
    [~,angular_velocity_jacobian,linear_velocity_jacobian]=compute_velocity_from_xdot(vec_x_ref2base_dot_matlab,current_base);
    corin_angular_velocity_jacobian(:,ii) = angular_velocity_jacobian;
    corin_linear_velocity_jacobian(:,ii) = linear_velocity_jacobian;
    
    
    corin_q_vec(:,ii) = corin_q(9:26);
    
    
    %% compute the control input
    [corin_u,norm_of_whole_error,norm_jacobian] = corin_controller.compute_setpoint_control_signal(corin_q,Xd,x_ref2f1_dot);
    %     corin_u = corin_controller.compute_setpoint_control_signal(corin_q,Xd);
    corin_q(9:26) = corin_q(9:26)+corin_u * sampling_time;
    
    corin_q_plus_u(:,ii) = corin_q(9:26);
    
    if ii == 501
        ref2f1 = vi.get_object_pose('/hexapod_new/footTip0');
        ref2base = vi.get_object_pose('/hexapod_new/body');
        f12base = ref2f1' * ref2base;
        q = corin_hexapod_new.get_q_from_vrep();
    end
    
    norm_of_u_leg(:,ii) = norm(corin_u);
    norm_of_u_leg1(:,ii) = norm(corin_u(1:3));
    norm_of_u_leg2(:,ii) = norm(corin_u(4:6));
    norm_of_u_leg3(:,ii) = norm(corin_u(7:9));
    norm_of_u_leg4(:,ii) = norm(corin_u(10:12));
    norm_of_u_leg5(:,ii) = norm(corin_u(13:15));
    norm_of_u_leg6(:,ii) = norm(corin_u(16:18));
    
    %% Send desired values
        corin_hexapod.send_target_joint_to_vrep(corin_q(9:26));
%     corin_hexapod.send_q_to_vrep(corin_q(9:26));
    
    %% update last value
    last_corin_foothold1 = current_corin_foothold1;
    last_base = current_base;
    result_constraint = Constraint_matrix * corin_u - Constraint_Vector;
    last_leg1_q = current_leg1_q;
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
    corin_whole_tracking_error_norm(:,ii) =norm_of_whole_error;
    corin_jacobian_error_norm(:,ii) = norm_jacobian;
    
    corin_control_inputs(:,ii) = corin_u;
    corin_q_vector(:,ii) = corin_q;
    task_error = corin_controller.get_last_error_signal();
    corin_absolute_error_norm(:,ii) = norm(task_error(1:8));
    corin_relative_error_norm(:,ii) = norm(task_error(9:48));
    corin_relative1_error_norm(:,ii) = norm(task_error(9:16));
    corin_relative2_error_norm(:,ii) = norm(task_error(17:24));
    corin_relative3_error_norm(:,ii) = norm(task_error(25:32));
    corin_relative4_error_norm(:,ii) = norm(task_error(33:40));
    corin_relative5_error_norm(:,ii) = norm(task_error(41:48));
    
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
% figure(1)
% plot(T,corin_whole_tracking_error_norm)
% title('norm of corin whole tracking error')
% xlabel('Time/s')
% ylabel('corin whole tracking error norm')

figure(1)
plot(T,corin_tracking_error_norm)
title('norm of corin tracking error')
xlabel('Time/s')
ylabel('corin tracking error norm')

% figure(3)
% plot(T,corin_jacobian_error_norm)
% title('norm of corin jacobian tracking error')
% xlabel('Time/s')
% ylabel('corin tracking error norm')

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
plot(T,corin_relative1_error_norm)
title('norm of corin relative pose 1 error')
xlabel('Time/s')
ylabel('corin relative pose 1 error norm')

figure(5)
plot(T,corin_relative2_error_norm)
title('norm of corin relative pose 2 error')
xlabel('Time/s')
ylabel('corin relative pose 2 error norm')

figure(6)
plot(T,corin_relative3_error_norm)
title('norm of corin relative pose 3 error')
xlabel('Time/s')
ylabel('corin relative pose 3 error norm')

figure(7)
plot(T,norm_of_u_leg)
hold on;
plot(T,norm_of_u_leg1)
hold on;
plot(T,norm_of_u_leg2)
hold on;
plot(T,norm_of_u_leg3)
hold on;
plot(T,norm_of_u_leg4)
hold on;
plot(T,norm_of_u_leg5)
hold on;
plot(T,norm_of_u_leg6)
title('norm of control input')
xlabel('Time/s')
ylabel('norm of control input')
legend('norm of control input','norm of control input for leg1','norm of control input for leg2',...
    'norm of control input for leg3','norm of control input for leg4','norm of control input for leg5',...
    'norm of control input for leg6')

figure(8)
plot(T,corin_relative_error_norm)
hold on;
plot(T,corin_relative1_error_norm)
hold on;
plot(T,corin_relative2_error_norm)
hold on;
plot(T,corin_relative3_error_norm)
hold on;
plot(T,corin_relative4_error_norm)
hold on;
plot(T,corin_relative5_error_norm)
title('norm of corin relative pose error')
xlabel('Time/s')
ylabel('corin relative pose error norm')
legend('norm of corin relative pose error','norm of corin relative pose 1 error','norm of corin relative pose 2 error',...
    'norm of corin relative pose 3 error','norm of corin relative pose 4 error','norm of corin relative pose 5 error')


% figure(6)
% plot(T,corin_base_angular_velocity_vrep(1,:))
% hold on;
% plot(T,corin_base_angular_velocity_vrep(2,:))
% hold on;
% plot(T,corin_base_angular_velocity_vrep(3,:))
% hold on;
% plot(T,corin_base_angular_velocity_matlab(1,:),':')
% hold on;
% plot(T,corin_base_angular_velocity_matlab(2,:),':')
% hold on;
% plot(T,corin_base_angular_velocity_matlab(3,:),':')
% hold on;
% title('velocity')
% xlabel('Time/s')
% ylabel('velocity from vrep and matlab')
% legend('angular velocity from vrep rx','angular velocity from vrep ry','angular velocity from vrep rz',...
%     'angular velocity from matlab rx','angular velocity from matlab ry','angular velocity from matlab rz')
% 
% 
% figure(7)
% plot(T,corin_base_linear_velocity_vrep(1,:))
% hold on;
% plot(T,corin_base_linear_velocity_vrep(2,:))
% hold on;
% plot(T,corin_base_linear_velocity_vrep(3,:))
% hold on;
% plot(T,corin_base_linear_velocity_matlab(1,:),':')
% hold on;
% plot(T,corin_base_linear_velocity_matlab(2,:),':')
% hold on;
% plot(T,corin_base_linear_velocity_matlab(3,:),':')
% hold on;
% title('velocity')
% xlabel('Time/s')
% ylabel('velocity from vrep and matlab')
% legend('linear velocity from vrep x','linear velocity from vrep y','linear velocity from vrep z',...
%     'linear velocity from matlab x','linear velocity from matlab y','linear velocity from matlab z')
% 
% figure(8)
% plot(T,corin_linear_velocity_jacobian(1,:))
% hold on;
% plot(T,corin_linear_velocity_jacobian(2,:))
% hold on;
% plot(T,corin_linear_velocity_jacobian(3,:))
% hold on;
% plot(T,corin_base_linear_velocity_matlab(1,:),':')
% hold on;
% plot(T,corin_base_linear_velocity_matlab(2,:),':')
% hold on;
% plot(T,corin_base_linear_velocity_matlab(3,:),':')
% hold on;
% title('base linear velocity')
% xlabel('Time/s')
% ylabel('base linear velocity')
% legend('linear velocity from jacobian x','linear velocity from jacobian y','linear velocity from jacobian z',...
%     'linear velocity from matlab x','linear velocity from matlab y','linear velocity from matlab z')
% 

% figure(6)
% plot(T,corin_angular_velocity_jacobian(1,:))
% hold on;
% plot(T,corin_angular_velocity_jacobian(2,:))
% hold on;
% plot(T,corin_angular_velocity_jacobian(3,:))
% hold on;
% plot(T,corin_base_angular_velocity_matlab(1,:),':')
% hold on;
% plot(T,corin_base_angular_velocity_matlab(2,:),':')
% hold on;
% plot(T,corin_base_angular_velocity_matlab(3,:),':')
% hold on;
% title('base angular velocity')
% xlabel('Time/s')
% ylabel('angular velocity')
% legend('angular velocity from jacobian rx','angular velocity from jacobian ry','angular velocity from jacobian rz',...
%     'angular velocity from matlab rx','angular velocity from matlab ry','angular velocity from matlab rz')
% 
% figure(10);
% plot(T,norm_of_the_erroe_x_ref2_base_dot_matlab_and_vrep)
% title('norm of the error for xref2basedot on matlab and vrep')
% xlabel('Time/s')

% %-------------------------------control input plot ------------------------
figure(9)
plot(T,corin_control_inputs(1,:))
hold on
plot(T,corin_control_inputs(2,:))
hold on
plot(T,corin_control_inputs(3,:))
hold on
title('control input u for leg1')
xlabel('Time/s')
ylabel('control input u for leg1')
legend('control input u of joint 1 in leg 1','control input u of joint 2 in leg 1','control input u of joint 3 in leg 1')


figure(10)
plot(T,corin_control_inputs(4,:))
hold on
plot(T,corin_control_inputs(5,:))
hold on
plot(T,corin_control_inputs(6,:))
hold on
title('control input u for leg2')
xlabel('Time/s')
ylabel('control input u for leg2')
legend('control input u of joint 1 in leg 2','control input u of joint 2 in leg 2','control input u of joint 3 in leg 2')

figure(11)
plot(T,corin_control_inputs(7,:))
hold on
plot(T,corin_control_inputs(8,:))
hold on
plot(T,corin_control_inputs(9,:))
hold on
title('control input u for leg3')
xlabel('Time/s')
ylabel('control input u for leg3')
legend('control input u of joint 1 in leg 3','control input u of joint 2 in leg 3','control input u of joint 3 in leg 3')

figure(12)
plot(T,corin_control_inputs(10,:))
hold on
plot(T,corin_control_inputs(11,:))
hold on
plot(T,corin_control_inputs(12,:))
hold on
title('control input u for leg4')
xlabel('Time/s')
ylabel('control input u for leg1')
legend('control input u of joint 1 in leg 4','control input u of joint 2 in leg 4','control input u of joint 3 in leg 4')

figure(13)
plot(T,corin_control_inputs(13,:))
hold on
plot(T,corin_control_inputs(14,:))
hold on
plot(T,corin_control_inputs(15,:))
hold on
title('control input u for leg5')
xlabel('Time/s')
ylabel('control input u for leg5')
legend('control input u of joint 1 in leg 5','control input u of joint 2 in leg 5','control input u of joint 3 in leg 5')

figure(14)
plot(T,corin_control_inputs(16,:))
hold on
plot(T,corin_control_inputs(17,:))
hold on
plot(T,corin_control_inputs(18,:))
hold on
title('control input u for leg6')
xlabel('Time/s')
ylabel('control input u for leg6')
legend('control input u of joint 1 in leg 6','control input u of joint 2 in leg 6','control input u of joint 3 in leg 6')

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
% %_____________________________________________________________________________

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
figure(21)
plot(T,linear_velocity_foothold1_vrep(1,:));
hold on;
plot(T,linear_velocity_foothold1_vrep(2,:));
hold on;
plot(T,linear_velocity_foothold1_vrep(3,:));
hold on;
title('linear velocity foothold1')
xlabel('Time/s')
ylabel('linear velocity foothold1')
legend( 'linear velicity foothold1 vrep x','linear velicity foothold1 vrep y','linear velicity foothold1 vrep z');

figure(22)
plot(T,linear_velocity_foothold2_vrep(1,:));
hold on;
plot(T,linear_velocity_foothold2_vrep(2,:));
hold on;
plot(T,linear_velocity_foothold2_vrep(3,:));
hold on;
title('linear velocity foothold2')
xlabel('Time/s')
ylabel('linear velocity foothold2')
legend( 'linear velicity foothold2 vrep x','linear velicity foothold2 vrep y','linear velicity foothold2 vrep z');
figure(23)
plot(T,linear_velocity_foothold3_vrep(1,:));
hold on;
plot(T,linear_velocity_foothold3_vrep(2,:));
hold on;
plot(T,linear_velocity_foothold3_vrep(3,:));
hold on;
title('linear velocity foothold3')
xlabel('Time/s')
ylabel('linear velocity foothold3')
legend( 'linear velicity foothold3 vrep x','linear velicity foothold3 vrep y','linear velicity foothold3 vrep z');
figure(24)
plot(T,linear_velocity_foothold4_vrep(1,:));
hold on;
plot(T,linear_velocity_foothold4_vrep(2,:));
hold on;
plot(T,linear_velocity_foothold4_vrep(3,:));
hold on;
title('linear velocity foothold4')
xlabel('Time/s')
ylabel('linear velocity foothold4')
legend( 'linear velicity foothold4 vrep x','linear velicity foothold4 vrep y','linear velicity foothold4 vrep z');
figure(25)
plot(T,linear_velocity_foothold5_vrep(1,:));
hold on;
plot(T,linear_velocity_foothold5_vrep(2,:));
hold on;
plot(T,linear_velocity_foothold5_vrep(3,:));
hold on;
title('linear velocity foothold5')
xlabel('Time/s')
ylabel('linear velocity foothold5')
legend( 'linear velicity foothold5 vrep x','linear velicity foothold5 vrep y','linear velicity foothold5 vrep z');
figure(26)
plot(T,linear_velocity_foothold6_vrep(1,:));
hold on;
plot(T,linear_velocity_foothold6_vrep(2,:));
hold on;
plot(T,linear_velocity_foothold6_vrep(3,:));
hold on;
title('linear velocity foothold6')
xlabel('Time/s')
ylabel('linear velocity foothold6')
legend( 'linear velicity foothold6 vrep x','linear velicity foothold6 vrep y','linear velicity foothold6 vrep z');

figure(27)
plot(T,rank_jacobian);
title('rank of jacobian matrix');
legend('rank of jacobian matrix')
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