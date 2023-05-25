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

function [Foothold2_to_foothold1_trans, Foothold3_to_foothold1_trans, Foothold4_to_foothold1_trans, Foothold5_to_foothold1_trans, Foothold6_to_foothold1_trans,...
    Foothold1_to_foothold2_matlab_trans,Foothold1_to_foothold3_matlab_trans,Foothold1_to_foothold4_matlab_trans,Foothold1_to_foothold5_matlab_trans,...
    Foothold1_to_foothold6_matlab_trans,Body_copp_trans,Body_matlab_trans,Norm_of_error]= vrep_hexapodscene_whole_body_relative_jacobian_test(simulation_parameters)

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
corin_hexapod = DQVrepHexapod('hexapod',vi);
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

Foothold2_to_foothold1_trans = zeros(3,max_iterations);
Foothold3_to_foothold1_trans = zeros(3,max_iterations);
Foothold4_to_foothold1_trans = zeros(3,max_iterations);
Foothold5_to_foothold1_trans = zeros(3,max_iterations);
Foothold6_to_foothold1_trans = zeros(3,max_iterations);
Body_copp_trans = zeros(3,max_iterations);

Foothold1_to_foothold2_rotation = zeros(4,max_iterations);
Foothold1_to_foothold3_rotation = zeros(4,max_iterations);
Foothold1_to_foothold4_rotation = zeros(4,max_iterations);
Foothold1_to_foothold5_rotation = zeros(4,max_iterations);
Foothold1_to_foothold6_rotation = zeros(4,max_iterations);
Body_copp_rotation = zeros(4,max_iterations);

Foothold1_to_foothold2_matlab_trans = zeros(3,max_iterations);
Foothold1_to_foothold3_matlab_trans = zeros(3,max_iterations);
Foothold1_to_foothold4_matlab_trans = zeros(3,max_iterations);
Foothold1_to_foothold5_matlab_trans = zeros(3,max_iterations);
Foothold1_to_foothold6_matlab_trans = zeros(3,max_iterations);
Body_matlab_trans = zeros(3,max_iterations);

Foothold1_to_foothold2_matlab_rotation = zeros(4,max_iterations);
Foothold1_to_foothold3_matlab_rotation = zeros(4,max_iterations);
Foothold1_to_foothold4_matlab_rotation = zeros(4,max_iterations);
Foothold1_to_foothold5_matlab_rotation = zeros(4,max_iterations);
Foothold1_to_foothold6_matlab_rotation = zeros(4,max_iterations);
Body_matlab_rotation = zeros(4,max_iterations);

Vec_norm_error = zeros(1,max_iterations);


linear_velocity_r1_reference_vrep = zeros(3,max_iterations);
linear_velocity_r2_reference_vrep = zeros(3,max_iterations);
linear_velocity_r3_reference_vrep = zeros(3,max_iterations);
linear_velocity_r4_reference_vrep = zeros(3,max_iterations);
linear_velocity_r5_reference_vrep = zeros(3,max_iterations);

linear_velocity_foothold2_reference_vrep = zeros(3,max_iterations);

linear_velocity_r1_reference_vrep_API = zeros(3,max_iterations);
linear_velocity_r2_reference_vrep_API = zeros(3,max_iterations);
linear_velocity_r3_reference_vrep_API = zeros(3,max_iterations);
linear_velocity_r4_reference_vrep_API = zeros(3,max_iterations);
linear_velocity_r5_reference_vrep_API = zeros(3,max_iterations);

angular_velocity_r1_reference_vrep_API = zeros(3,max_iterations);
angular_velocity_r2_reference_vrep_API = zeros(3,max_iterations);
angular_velocity_r3_reference_vrep_API = zeros(3,max_iterations);
angular_velocity_r4_reference_vrep_API = zeros(3,max_iterations);
angular_velocity_r5_reference_vrep_API = zeros(3,max_iterations);

linear_velocity_r1_jacobian = zeros(3,max_iterations);
linear_velocity_r2_jacobian = zeros(3,max_iterations);
linear_velocity_r3_jacobian = zeros(3,max_iterations);
linear_velocity_r4_jacobian = zeros(3,max_iterations);
linear_velocity_r5_jacobian = zeros(3,max_iterations);

     angular_velocity_r1_jacobian = zeros(3,max_iterations);
    angular_velocity_r2_jacobian = zeros(3,max_iterations);
    angular_velocity_r3_jacobian = zeros(3,max_iterations);
    angular_velocity_r4_jacobian = zeros(3,max_iterations);
    angular_velocity_r5_jacobian = zeros(3,max_iterations);



linear_velocity_foothold1_approximation = zeros(3,max_iterations);
linear_velocity_foothold1_vrep = zeros(3,max_iterations);
angular_velocity_foothold1_approximation = zeros(3,max_iterations);
angular_velocity_foothold1_vrep = zeros(3,max_iterations);

vec_linear_velocity_body_vrep = zeros(3,max_iterations);
vec_angular_velocity_body_vrep = zeros(3,max_iterations);
vec_linear_velocity_body_jacobian = zeros(3,max_iterations);
vec_angular_velocity_body_jacobian = zeros(3,max_iterations);
vec_linear_velocity_body_approximation = zeros(3,max_iterations);
vec_angular_velocity_body_approximation = zeros(3,max_iterations);
vec_x_ref2base_dot_approximation = zeros(8,max_iterations);
vec_x_ref2base_dot_jacobian = zeros(8,max_iterations);

current_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0');
last_corin_foothold1 = vi.get_object_pose('/hexapod/footTip0');
Norm_of_error = [];

ii = 1;
    last_foothold2 = vi.get_object_pose('/hexapod/footTip1',-1,vi.OP_BLOCKING);
    foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
    foothold2 = vi.get_object_pose('/hexapod/footTip1',-1,vi.OP_BLOCKING);
    foothold3 = vi.get_object_pose('/hexapod/footTip2',-1,vi.OP_BLOCKING);
    foothold4 = vi.get_object_pose('/hexapod/footTip3',-1,vi.OP_BLOCKING);
    foothold5 = vi.get_object_pose('/hexapod/footTip4',-1,vi.OP_BLOCKING);
    foothold6 = vi.get_object_pose('/hexapod/footTip5',-1,vi.OP_BLOCKING);
    body_copp = corin_hexapod.get_body_pose_from_vrep();
    
      last_foothold2_to_foothold1_trans =  get_relative_translation_vector(foothold2,foothold1);
        last_foothold2_to_reference_trans = foothold2.translation;
        last_foothold2_to_reference_trans = vec4(last_foothold2_to_reference_trans);
        last_foothold2_to_reference_trans = last_foothold2_to_reference_trans(2:4);
%       last_foothold2_to_foothold1_trans = vi.get_object_pose('/hexapod/footTip1',-1,vi.OP_BLOCKING);
      last_foothold2_to_foothold3_trans =  get_relative_translation_vector(foothold2,foothold3);
      last_foothold2_to_foothold4_trans =  get_relative_translation_vector(foothold2,foothold4);
      last_foothold2_to_foothold5_trans =  get_relative_translation_vector(foothold2,foothold5);
      last_foothold2_to_foothold6_trans =  get_relative_translation_vector(foothold2,foothold6);
last_body_pose = vi.get_object_pose('/hexapod/body',-1,vi.OP_BLOCKING);
%     foothold1 = corin_hexapod.get_reference_feethold1_from_vrep;
% %     corin.set_reference_to_first_feethold(corin_q);
% 
%     corin.set_reference_to_reference_foothold_vrep(foothold1);
    a = 1;
    corin_q = corin_hexapod.get_q_from_vrep();
    last_leg_q = corin_q(9:26);
    
    
%% Control loop
for t=0:sampling_time:total_time
    %% get information from CoppeliaSim
    vi.synchronous_trigger();
        foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
%     foothold1 = corin_hexapod.get_reference_feethold1_from_vrep;
%     corin.set_reference_to_first_feethold(corin_q);
    corin.set_reference_to_reference_foothold_vrep(foothold1);
            corin_q = corin_hexapod.get_q_from_vrep();
    current_leg_q = corin_q(9:26);
    leg_q_dot = (current_leg_q - last_leg_q)/sampling_time;   
     Jacobian_whole_boody = corin.pose_jacobian(corin_q);
     corin.fkm(corin_q);
     
%     foothold1 = vi.get_object_pose('/hexapod/footTip0',-1,vi.OP_BLOCKING);
    foothold2 = vi.get_object_pose('/hexapod/footTip1',-1,vi.OP_BLOCKING);
    foothold3 = vi.get_object_pose('/hexapod/footTip2',-1,vi.OP_BLOCKING);
    foothold4 = vi.get_object_pose('/hexapod/footTip3',-1,vi.OP_BLOCKING);
    foothold5 = vi.get_object_pose('/hexapod/footTip4',-1,vi.OP_BLOCKING);
    foothold6 = vi.get_object_pose('/hexapod/footTip5',-1,vi.OP_BLOCKING);
    body_copp = vi.get_object_pose('/hexapod/body',-1,vi.OP_BLOCKING);
    
    %---------可能有问题--------------------
        foothold2_to_foothold1 = foothold2' * foothold1;
    foothold2_to_foothold3 = foothold2' * foothold2;
    foothold2_to_foothold4 = foothold2' * foothold3;
    foothold2_to_foothold5 = foothold2' * foothold4;
    foothold2_to_foothold6 = foothold2' * foothold5; 
    
    
            foothold1_to_foothold2 = foothold1' * foothold2;
    foothold1_to_foothold3 = foothold1' * foothold3;
    foothold1_to_foothold4 = foothold1' * foothold4;
    foothold1_to_foothold5 = foothold1' * foothold5;
    foothold1_to_foothold6 = foothold1' * foothold6; 
    %--------------------------------------
      
foothold2_to_reference_trans = foothold2.translation;
foothold2_to_reference_trans = vec4(foothold2_to_reference_trans);
foothold2_to_reference_trans = foothold2_to_reference_trans(2:4);

 if ii == 15
     a = 0;
 end
      foothold2_to_foothold1_trans =  get_relative_translation_vector(foothold2,foothold1);
      foothold2_to_foothold3_trans =  get_relative_translation_vector(foothold2,foothold3);
      foothold2_to_foothold4_trans =  get_relative_translation_vector(foothold2,foothold4);
      foothold2_to_foothold5_trans =  get_relative_translation_vector(foothold2,foothold5);
      foothold2_to_foothold6_trans =  get_relative_translation_vector(foothold2,foothold6);
      
   %---------------------relative linear velocity -----------------------
      linear_velocity_f2toref = (foothold2_to_reference_trans - last_foothold2_to_reference_trans)/sampling_time;
      
      linear_velocity_f2tof1 = (foothold2_to_foothold1_trans - last_foothold2_to_foothold1_trans)/sampling_time;
      linear_velocity_f3tof1 = (foothold2_to_foothold3_trans - last_foothold2_to_foothold3_trans)/sampling_time;
      linear_velocity_f4tof1 = (foothold2_to_foothold4_trans - last_foothold2_to_foothold4_trans)/sampling_time;
      linear_velocity_f5tof1 = (foothold2_to_foothold5_trans - last_foothold2_to_foothold5_trans)/sampling_time;
      linear_velocity_f6tof1 = (foothold2_to_foothold6_trans - last_foothold2_to_foothold6_trans)/sampling_time;
      
      
      linear_velocity_foothold2_reference_vrep(:,ii) = linear_velocity_f2toref;
      linear_velocity_r1_reference_vrep(:,ii) = linear_velocity_f2tof1;
      linear_velocity_r2_reference_vrep(:,ii) = linear_velocity_f3tof1;
      linear_velocity_r3_reference_vrep(:,ii) = linear_velocity_f4tof1;
      linear_velocity_r4_reference_vrep(:,ii) = linear_velocity_f5tof1;
      linear_velocity_r5_reference_vrep(:,ii) = linear_velocity_f6tof1;
  %----------------------relative rotation-----------------------    
    body_copp_trans4 = vec4(body_copp.translation);
    body_copp_trans = body_copp_trans4(2:4);
    
    
    Foothold2_to_foothold1_trans(:,ii) = foothold2_to_foothold1_trans;
    Foothold3_to_foothold1_trans(:,ii) = foothold2_to_foothold3_trans;
    Foothold4_to_foothold1_trans(:,ii) = foothold2_to_foothold4_trans;
    Foothold5_to_foothold1_trans(:,ii) = foothold2_to_foothold5_trans;
    Foothold6_to_foothold1_trans(:,ii) = foothold2_to_foothold6_trans;
    Body_copp_trans(:,ii) = body_copp_trans';
    
    foothold1_to_foothold2_rot = foothold2_to_foothold1.rotation;
    foothold1_to_foothold3_rot = foothold2_to_foothold3.rotation;
    foothold1_to_foothold4_rot = foothold2_to_foothold4.rotation;
    foothold1_to_foothold5_rot = foothold2_to_foothold5.rotation;
    foothold1_to_foothold6_rot = foothold2_to_foothold6.rotation;
    body_copp_rot = body_copp.rotation;
    
    Foothold1_to_foothold2_rotation(:,ii) = vec4(foothold1_to_foothold2_rot);
    Foothold1_to_foothold3_rotation(:,ii) = vec4(foothold1_to_foothold3_rot);
    Foothold1_to_foothold4_rotation(:,ii) = vec4(foothold1_to_foothold4_rot);
    Foothold1_to_foothold5_rotation(:,ii) = vec4(foothold1_to_foothold5_rot);
    Foothold1_to_foothold6_rotation(:,ii) = vec4(foothold1_to_foothold6_rot);
    Body_copp_rotation(:,ii) = vec4(body_copp_rot);
    
    %--------------------foothold1 velocity approximation and vrep------------------------------
    [f1_dot_approximation,foothold1_angular_velocity_approximation,foothold1_linear_velocity_approximation] = compute_estimated_velocity(foothold2,last_foothold2,sampling_time);
linear_velocity_foothold1_approximation(:,ii) = foothold1_linear_velocity_approximation;
angular_velocity_foothold1_approximation(:,ii) = foothold1_angular_velocity_approximation;

[linear_velocity_f1_vrep,angular_velocity_f1_vrep] = vi.get_body_velocity_from_vrep('/hexapod/footTip0');
linear_velocity_foothold1_vrep(:,ii) = linear_velocity_f1_vrep;
angular_velocity_foothold1_vrep(:,ii) = angular_velocity_f1_vrep;

    %-------------------base velocity jacobian and vrep and approximation--------------------
    [linear_velocity_body_vrep,angular_velocity_body_vrep] = vi.get_body_velocity_from_vrep('/hexapod/body');
vec_linear_velocity_body_vrep(:,ii) = linear_velocity_body_vrep;
vec_angular_velocity_body_vrep(:,ii) = angular_velocity_body_vrep;

jacobian_abs = Jacobian_whole_boody(1:8,1:11);
jacobian_abs(:,9:11) = Jacobian_whole_boody(1:8,12:14);
vector_abs = zeros(11,1);
vector_abs(1:8) = vec8(f1_dot_approximation);
vector_abs(9:11) = leg_q_dot(4:6);
vec_base_dot = jacobian_abs*vector_abs;
current_body_pose = vi.get_object_pose('/hexapod/body',-1,vi.OP_BLOCKING);
[~,body_angular_velocity_jacobian, body_linear_velocity_jacobian] = compute_velocity_from_xdot(DQ(vec_base_dot),current_body_pose);

vec_linear_velocity_body_jacobian(:,ii) = body_linear_velocity_jacobian;
vec_angular_velocity_body_jacobian(:,ii) = body_angular_velocity_jacobian;

[x_ref2base_dot_approximation,~,~] = compute_estimated_velocity(current_body_pose,last_body_pose,sampling_time);
[~,body_angular_velocity_approximation, body_linear_velocity_approximation] = compute_velocity_from_xdot(x_ref2base_dot_approximation,current_body_pose);
vec_linear_velocity_body_approximation(:,ii) = body_linear_velocity_approximation;
vec_angular_velocity_body_approximation(:,ii) = body_angular_velocity_approximation;

vec_x_ref2base_dot_approximation(:,ii) = vec8(x_ref2base_dot_approximation);
vec_x_ref2base_dot_jacobian(:,ii) = vec_base_dot;


    %---------------- get realtive task velocity from vrep using api-------------------
    [linear_r1_v_vrep,angular_r1_w_vrep] = vi.get_body_velocity_from_vrep('/hexapod/footTip1');
    [linear_r2_v_vrep,angular_r2_w_vrep] = vi.get_body_velocity_from_vrep('/hexapod/footTip2');
    [linear_r3_v_vrep,angular_r3_w_vrep] = vi.get_body_velocity_from_vrep('/hexapod/footTip3');
    [linear_r4_v_vrep,angular_r4_w_vrep] = vi.get_body_velocity_from_vrep('/hexapod/footTip4');
    [linear_r5_v_vrep,angular_r5_w_vrep] = vi.get_body_velocity_from_vrep('/hexapod/footTip5');
    
    linear_velocity_r1_reference_vrep_API(:,ii) = linear_r1_v_vrep;
    linear_velocity_r2_reference_vrep_API(:,ii) = linear_r2_v_vrep;
    linear_velocity_r3_reference_vrep_API(:,ii) = linear_r3_v_vrep;
    linear_velocity_r4_reference_vrep_API(:,ii) = linear_r4_v_vrep;
    linear_velocity_r5_reference_vrep_API(:,ii) = linear_r5_v_vrep;
    
    angular_velocity_r1_reference_vrep_API(:,ii) = angular_r1_w_vrep;
angular_velocity_r2_reference_vrep_API(:,ii) = angular_r2_w_vrep;
angular_velocity_r3_reference_vrep_API(:,ii) = angular_r3_w_vrep;
angular_velocity_r4_reference_vrep_API(:,ii) = angular_r4_w_vrep;
angular_velocity_r5_reference_vrep_API(:,ii) = angular_r5_w_vrep;
    
    
    %% get information from Matlab

   
    
   
    
    
%-----------------relative task linear velocity jacobian----------------
     Jacobian_relative = Jacobian_whole_boody(9:48,9:26);
    x_relative_task_dot = Jacobian_relative*leg_q_dot;
    x_relative_task_dot_1 = x_relative_task_dot(1:8);
    x_relative_task_dot_2 = x_relative_task_dot(9:16);
    x_relative_task_dot_3 = x_relative_task_dot(17:24);
    x_relative_task_dot_4 = x_relative_task_dot(25:32);
    x_relative_task_dot_5 = x_relative_task_dot(33:40);
    
    
%     body_matlab = corin.fkm(corin_q,1);

    
    matlab_fkm_res = corin.fkm(corin_q);
    body_matlab = matlab_fkm_res(1);
    foothold1_to_foothold2_matlab = matlab_fkm_res(2);
    foothold1_to_foothold3_matlab = matlab_fkm_res(3);
    foothold1_to_foothold4_matlab = matlab_fkm_res(4);
    foothold1_to_foothold5_matlab = matlab_fkm_res(5);
    foothold1_to_foothold6_matlab = matlab_fkm_res(6);
    
    [~,angular_velocity_r1, linear_velocity_r1] = compute_velocity_from_xdot(x_relative_task_dot_1,foothold1_to_foothold2_matlab);
    [~,angular_velocity_r2, linear_velocity_r2] = compute_velocity_from_xdot(x_relative_task_dot_2,foothold1_to_foothold3_matlab);
    [~,angular_velocity_r3, linear_velocity_r3] = compute_velocity_from_xdot(x_relative_task_dot_3,foothold1_to_foothold4_matlab);
    [~,angular_velocity_r4, linear_velocity_r4] = compute_velocity_from_xdot(x_relative_task_dot_4,foothold1_to_foothold5_matlab);
    [~,angular_velocity_r5, linear_velocity_r5] = compute_velocity_from_xdot(x_relative_task_dot_5,foothold1_to_foothold6_matlab);
    
    linear_velocity_r1_jacobian(:,ii) = linear_velocity_r1;
    linear_velocity_r2_jacobian(:,ii) = linear_velocity_r2;
    linear_velocity_r3_jacobian(:,ii) = linear_velocity_r3;
    linear_velocity_r4_jacobian(:,ii) = linear_velocity_r4;
    linear_velocity_r5_jacobian(:,ii) = linear_velocity_r5;
    
        angular_velocity_r1_jacobian(:,ii) = angular_velocity_r1;
    angular_velocity_r2_jacobian(:,ii) = angular_velocity_r2;
    angular_velocity_r3_jacobian(:,ii) = angular_velocity_r3;
    angular_velocity_r4_jacobian(:,ii) = angular_velocity_r4;
    angular_velocity_r5_jacobian(:,ii) = angular_velocity_r5;
    
    
    body_matlab_trans4 = vec4(body_matlab.translation);
    body_matlab_trans = body_matlab_trans4(2:4);
    foothold1_to_foothold2_matlab_trans4 = vec4(foothold1_to_foothold2_matlab.translation);
    foothold1_to_foothold2_matlab_trans = foothold1_to_foothold2_matlab_trans4(2:4);
    foothold1_to_foothold3_matlab_trans4 = vec4(foothold1_to_foothold3_matlab.translation);
    foothold1_to_foothold3_matlab_trans = foothold1_to_foothold3_matlab_trans4(2:4);
    foothold1_to_foothold4_matlab_trans4 = vec4(foothold1_to_foothold4_matlab.translation);
    foothold1_to_foothold4_matlab_trans = foothold1_to_foothold4_matlab_trans4(2:4);
    foothold1_to_foothold5_matlab_trans4 = vec4(foothold1_to_foothold5_matlab.translation);
    foothold1_to_foothold5_matlab_trans = foothold1_to_foothold5_matlab_trans4(2:4);
    foothold1_to_foothold6_matlab_trans4 = vec4(foothold1_to_foothold6_matlab.translation);
    foothold1_to_foothold6_matlab_trans = foothold1_to_foothold6_matlab_trans4(2:4);

    
    Foothold1_to_foothold2_matlab_trans(:,ii) = foothold1_to_foothold2_matlab_trans';
    Foothold1_to_foothold3_matlab_trans(:,ii) = foothold1_to_foothold3_matlab_trans';
    Foothold1_to_foothold4_matlab_trans(:,ii) = foothold1_to_foothold4_matlab_trans';
    Foothold1_to_foothold5_matlab_trans(:,ii) = foothold1_to_foothold5_matlab_trans';
    Foothold1_to_foothold6_matlab_trans(:,ii) = foothold1_to_foothold6_matlab_trans';
    Body_matlab_trans(:,ii) = body_matlab_trans';
    
    foothold1_to_foothold2_matlab_rot = foothold1_to_foothold2_matlab.rotation;
    foothold1_to_foothold3_matlab_rot = foothold1_to_foothold3_matlab.rotation;
    foothold1_to_foothold4_matlab_rot = foothold1_to_foothold4_matlab.rotation;
    foothold1_to_foothold5_matlab_rot = foothold1_to_foothold5_matlab.rotation;
    foothold1_to_foothold6_matlab_rot = foothold1_to_foothold6_matlab.rotation;
    body_matlab_rot = body_matlab.rotation;
    Foothold1_to_foothold2_matlab_rotation(:,ii) = - vec4(foothold1_to_foothold2_matlab_rot);
    Foothold1_to_foothold3_matlab_rotation(:,ii) = - vec4(foothold1_to_foothold3_matlab_rot);
    Foothold1_to_foothold4_matlab_rotation(:,ii) = vec4(foothold1_to_foothold4_matlab_rot);
    Foothold1_to_foothold5_matlab_rotation(:,ii) = vec4(foothold1_to_foothold5_matlab_rot);
    Foothold1_to_foothold6_matlab_rotation(:,ii) = vec4(foothold1_to_foothold6_matlab_rot);
    Body_matlab_rotation(:,ii) = vec4(body_matlab_rot);
    
    Coppeliasim_data = [foothold2_to_foothold1,foothold2_to_foothold3,foothold2_to_foothold4,foothold2_to_foothold5,...
        foothold2_to_foothold6,body_copp];
    Matlab_data = [foothold1_to_foothold2_matlab,foothold1_to_foothold3_matlab,foothold1_to_foothold4_matlab,foothold1_to_foothold5_matlab,...
        foothold1_to_foothold6_matlab,body_matlab];
    
    task_error = zeros(48,1);
    for i = 2:6
        foothold = Coppeliasim_data(i);
        foothold_matlab = Matlab_data(i);
        rot_vrep = foothold.rotation;
        rot = foothold_matlab.rotation;
        foothold_matlab_new = foothold_matlab;
%         if i >= 2
%             foothold_matlab_new = foothold_matlab;
%         elseif i <= 1
%             rot_new = -rot;
%             trans = foothold_matlab.translation;
%             foothold_matlab_new =  rot_new + E_*0.5*trans*rot_new;
%         else
%             error('error');
%         end
        one_task_error = vec8(foothold_matlab_new) - vec8(foothold);
        task_error(8*(i-1) +1 :8*i) = one_task_error;
    end
    
    
%     footTip0_velocity = compute_estimated_velocity(current_footTip0,last_footTip0,sampling_time);
    
    
    nor = norm(task_error);
    Vec_norm_error(ii,:) = nor;

if nor>2
    aaa = 0
end
    
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
    
    
    
    
    %% Send desired values
    if a == 310
        b = 0;
    end
    
    for j = 1:6
        if j ~=1
            if a <=450
                if corin_q(8+3*(j-1)+1) <= 0.3
                    corin_q(8+3*(j-1)+1) = corin_q(8+3*(j-1)+1) + 0.002*j;
                end
                if corin_q(8+3*(j-1)+2) <= 0
                    corin_q(8+3*(j-1)+2) = corin_q(8+3*(j-1)+2) + 0.002*j;
                end
                if corin_q(8+3*(j-1)+3) >= 0
                    corin_q(8+3*(j-1)+3) = corin_q(8+3*(j-1)+3) - 0.005*j;
                end
            elseif a >=450
                if corin_q(8+3*(j-1)+1) >= 0
                    corin_q(8+3*(j-1)+1) = corin_q(8+3*(j-1)+1) - 0.002*j;
                end
                if corin_q(8+3*(j-1)+2) >= -0.5236
                    corin_q(8+3*(j-1)+2) = corin_q(8+3*(j-1)+2) - 0.002*j;
                end
                if corin_q(8+3*(j-1)+3) <= 2.0942
                    corin_q(8+3*(j-1)+3) = corin_q(8+3*(j-1)+3) + 0.005*j;
                end
            end
        end
    end

    
%     corin_hexapod.send_q_to_vrep(corin_q(9:26));
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
    
    %% get data to plot
    last_foothold2_to_foothold1_trans = foothold2_to_foothold1_trans;
    last_foothold2_to_foothold3_trans = foothold2_to_foothold3_trans;
    last_foothold2_to_foothold4_trans = foothold2_to_foothold4_trans;
    last_foothold2_to_foothold5_trans = foothold2_to_foothold5_trans;
    last_foothold2_to_foothold6_trans = foothold2_to_foothold6_trans;
    last_foothold2_to_reference_trans = foothold2_to_reference_trans;
    last_foothold2 = foothold2;
    last_body_pose=current_body_pose;
    last_leg_q = current_leg_q;
    ii = ii + 1;
    
    
end
T=0:sampling_time:total_time;
figure(1)
plot(T,Foothold2_to_foothold1_trans(1,:))
hold on;
plot(T,Foothold2_to_foothold1_trans(2,:))
hold on;
plot(T,Foothold2_to_foothold1_trans(3,:))
hold on;
plot(T,Foothold1_to_foothold2_matlab_trans(1,:),':')
hold on;
plot(T,Foothold1_to_foothold2_matlab_trans(2,:),':')
hold on;
plot(T,Foothold1_to_foothold2_matlab_trans(3,:),':')
hold on;
title('foothold 1 to 2 translation')
xlabel('Time/s') 
ylabel('foothold 1 to 2 translation from vrep and matlab') 
legend('vrep x','vrep y','vrep z',...
    'matlab x','matlab y','matlab z');
figure(2)
plot(T,Foothold3_to_foothold1_trans(1,:))
hold on;
plot(T,Foothold3_to_foothold1_trans(2,:))
hold on;
plot(T,Foothold3_to_foothold1_trans(3,:))
hold on;
plot(T,Foothold1_to_foothold3_matlab_trans(1,:),':')
hold on;
plot(T,Foothold1_to_foothold3_matlab_trans(2,:),':')
hold on;
plot(T,Foothold1_to_foothold3_matlab_trans(3,:),':')
hold on;
title('foothold 2 to 3 translation')
xlabel('Time/s') 
ylabel('foothold 2 to 3 translation from vrep and matlab') 
legend('vrep x','vrep y','vrep z',...
    'matlab x','matlab y','matlab z');
figure(3)
plot(T,Foothold4_to_foothold1_trans(1,:))
hold on;
plot(T,Foothold4_to_foothold1_trans(2,:))
hold on;
plot(T,Foothold4_to_foothold1_trans(3,:))
hold on;
plot(T,Foothold1_to_foothold4_matlab_trans(1,:),':')
hold on;
plot(T,Foothold1_to_foothold4_matlab_trans(2,:),':')
hold on;
plot(T,Foothold1_to_foothold4_matlab_trans(3,:),':')
hold on;
title('foothold 3 to 4 translation')
xlabel('Time/s') 
ylabel('foothold  3 to 4 translation from vrep and matlab') 
legend('vrep x','vrep y','vrep z',...
    'matlab x','matlab y','matlab z');
figure(4)
plot(T,Foothold5_to_foothold1_trans(1,:))
hold on;
plot(T,Foothold5_to_foothold1_trans(2,:))
hold on;
plot(T,Foothold5_to_foothold1_trans(3,:))
hold on;
plot(T,Foothold1_to_foothold5_matlab_trans(1,:),':')
hold on;
plot(T,Foothold1_to_foothold5_matlab_trans(2,:),':')
hold on;
plot(T,Foothold1_to_foothold5_matlab_trans(3,:),':')
hold on;
title('foothold 4 to 5 translation')
xlabel('Time/s') 
ylabel('foothold 4 to 5 translation from vrep and matlab') 
legend('vrep x','vrep y','vrep z',...
    'matlab x','matlab y','matlab z');
figure(5)
plot(T,Foothold6_to_foothold1_trans(1,:))
hold on;
plot(T,Foothold6_to_foothold1_trans(2,:))
hold on;
plot(T,Foothold6_to_foothold1_trans(3,:))
hold on;
plot(T,Foothold1_to_foothold6_matlab_trans(1,:),':')
hold on;
plot(T,Foothold1_to_foothold6_matlab_trans(2,:),':')
hold on;
plot(T,Foothold1_to_foothold6_matlab_trans(3,:),':')
hold on;
title('foothold 5 to 6 translation')
xlabel('Time/s') 
ylabel('foothold 5 to 6 translation from vrep and matlab') 
legend('vrep x','vrep y','vrep z',...
    'matlab x','matlab y','matlab z');
figure(6)
plot(T,Body_copp_trans(1,:))
hold on;
plot(T,Body_copp_trans(2,:))
hold on;
plot(T,Body_copp_trans(3,:))
hold on;
plot(T,Body_matlab_trans(1,:),':')
hold on;
plot(T,Body_matlab_trans(2,:),':')
hold on;
plot(T,Body_matlab_trans(3,:),':')
hold on;
title('body translation')
xlabel('Time/s') 
ylabel('body translation from vrep and matlab') 
legend('vrep x','vrep y','vrep z',...
    'matlab x','matlab y','matlab z');
figure(7)
plot(T,Foothold1_to_foothold2_rotation(1,:))
hold on;
plot(T,Foothold1_to_foothold2_rotation(2,:))
hold on;
plot(T,Foothold1_to_foothold2_rotation(3,:))
hold on;
plot(T,Foothold1_to_foothold2_rotation(4,:))
hold on;
plot(T,Foothold1_to_foothold2_matlab_rotation(1,:),':')
hold on;
plot(T,Foothold1_to_foothold2_matlab_rotation(2,:),':')
hold on;
plot(T,Foothold1_to_foothold2_matlab_rotation(3,:),':')
hold on;
plot(T,Foothold1_to_foothold2_matlab_rotation(4,:),':')
hold on;
title('foothold 1 to 2 rotaion')
xlabel('Time/s') 
ylabel('foothold 1 to 2 rotation from vrep and matlab') 
legend('vrep quaternion real','vrep quaternion i','vrep quaternion j','vrep quaternion k',...
    'matlab quaternion real','matlab quaternion i','matlab quaternion j','matlab quaternion k');
figure(8)
plot(T,Foothold1_to_foothold3_rotation(1,:))
hold on;
plot(T,Foothold1_to_foothold3_rotation(2,:))
hold on;
plot(T,Foothold1_to_foothold3_rotation(3,:))
hold on;
plot(T,Foothold1_to_foothold3_rotation(4,:))
hold on;
plot(T,Foothold1_to_foothold3_matlab_rotation(1,:),':')
hold on;
plot(T,Foothold1_to_foothold3_matlab_rotation(2,:),':')
hold on;
plot(T,Foothold1_to_foothold3_matlab_rotation(3,:),':')
hold on;
plot(T,Foothold1_to_foothold3_matlab_rotation(4,:),':')
hold on;
title('foothold 2 to 3 rotaion')
xlabel('Time/s') 
ylabel('foothold 2 to 3 rotation from vrep and matlab') 
legend('vrep quaternion real','vrep quaternion i','vrep quaternion j','vrep quaternion k',...
    'matlab quaternion real','matlab quaternion i','matlab quaternion j','matlab quaternion k');
figure(9)
plot(T,Foothold1_to_foothold4_rotation(1,:))
hold on;
plot(T,Foothold1_to_foothold4_rotation(2,:))
hold on;
plot(T,Foothold1_to_foothold4_rotation(3,:))
hold on;
plot(T,Foothold1_to_foothold4_rotation(4,:))
hold on;
plot(T,Foothold1_to_foothold4_matlab_rotation(1,:),':')
hold on;
plot(T,Foothold1_to_foothold4_matlab_rotation(2,:),':')
hold on;
plot(T,Foothold1_to_foothold4_matlab_rotation(3,:),':')
hold on;
plot(T,Foothold1_to_foothold4_matlab_rotation(4,:),':')
hold on;
title('foothold 4 to 5 rotation')
xlabel('Time/s') 
ylabel('foothold 3 to 4 rotation from vrep and matlab') 
legend('vrep quaternion real','vrep quaternion i','vrep quaternion j','vrep quaternion k',...
    'matlab quaternion real','matlab quaternion i','matlab quaternion j','matlab quaternion k');
figure(10)
plot(T,Foothold1_to_foothold5_rotation(1,:))
hold on;
plot(T,Foothold1_to_foothold5_rotation(2,:))
hold on;
plot(T,Foothold1_to_foothold5_rotation(3,:))
hold on;
plot(T,Foothold1_to_foothold5_rotation(4,:))
hold on;
plot(T,Foothold1_to_foothold5_matlab_rotation(1,:),':')
hold on;
plot(T,Foothold1_to_foothold5_matlab_rotation(2,:),':')
hold on;
plot(T,Foothold1_to_foothold5_matlab_rotation(3,:),':')
hold on;
plot(T,Foothold1_to_foothold5_matlab_rotation(4,:),':')
hold on;
title('foothold 5 to 6 rotation')
xlabel('Time/s') 
ylabel('foothold 4 to 5 rotation from vrep and matlab') 
legend('vrep quaternion real','vrep quaternion i','vrep quaternion j','vrep quaternion k',...
    'matlab quaternion real','matlab quaternion i','matlab quaternion j','matlab quaternion k');
figure(11)
plot(T,Foothold1_to_foothold6_rotation(1,:))
hold on;
plot(T,Foothold1_to_foothold6_rotation(2,:))
hold on;
plot(T,Foothold1_to_foothold6_rotation(3,:))
hold on;
plot(T,Foothold1_to_foothold6_rotation(4,:))
hold on;
plot(T,Foothold1_to_foothold6_matlab_rotation(1,:),':')
hold on;
plot(T,Foothold1_to_foothold6_matlab_rotation(2,:),':')
hold on;
plot(T,Foothold1_to_foothold6_matlab_rotation(3,:),':')
hold on;
plot(T,Foothold1_to_foothold6_matlab_rotation(4,:),':')
hold on;
title('body rotation')
xlabel('Time/s') 
ylabel('foothold 5 to 6 rotation from vrep and matlab') 
legend('vrep quaternion real','vrep quaternion i','vrep quaternion j','vrep quaternion k',...
    'matlab quaternion real','matlab quaternion i','matlab quaternion j','matlab quaternion k');
figure(12)
plot(T,Body_copp_rotation(1,:))
hold on;
plot(T,Body_copp_rotation(2,:))
hold on;
plot(T,Body_copp_rotation(3,:))
hold on;
plot(T,Body_copp_rotation(4,:))
hold on;
plot(T,Body_matlab_rotation(1,:),':')
hold on;
plot(T,Body_matlab_rotation(2,:),':')
hold on;
plot(T,Body_matlab_rotation(3,:),':')
hold on;
plot(T,Body_matlab_rotation(4,:),':')
hold on;
title('body rotation')
xlabel('Time/s') 
ylabel('body rotation from vrep and matlab') 
legend('vrep quaternion real','vrep quaternion i','vrep quaternion j','vrep quaternion k',...
    'matlab quaternion real','matlab quaternion i','matlab quaternion j','matlab quaternion k');

figure(13);
plot(T,Vec_norm_error);
title('norm of the error for task space pose on matlab and vrep')
xlabel('Time/s') 


figure(14)
plot(T,linear_velocity_r1_reference_vrep(1,:))
hold on;
plot(T,linear_velocity_r1_reference_vrep(2,:))
hold on;
plot(T,linear_velocity_r1_reference_vrep(3,:))
hold on;
plot(T,linear_velocity_r1_jacobian(1,:),':')
hold on;
plot(T,linear_velocity_r1_jacobian(2,:),':')
hold on;
plot(T,linear_velocity_r1_jacobian(3,:),':')
hold on;
plot(T,linear_velocity_r1_reference_vrep_API(1,:),'--')
hold on;
plot(T,linear_velocity_r1_reference_vrep_API(2,:),'--')
hold on;
plot(T,linear_velocity_r1_reference_vrep_API(3,:),'--')
hold on;
plot(T,linear_velocity_foothold2_reference_vrep(1,:),'.');
hold on;
plot(T,linear_velocity_foothold2_reference_vrep(2,:),'.');
hold on;
plot(T,linear_velocity_foothold2_reference_vrep(3,:),'.');
title('relative task1 linear velocity')
xlabel('Time/s') 
ylabel('relative task1 linear velocity from vrep and jacobian matlab') 
legend('relative task1 linear velocity x reference','relative task1 linear velocity y reference','relative task1 linear velocity z reference',...
    'relative task1 linear velocity x from jacobian','relative task1 linear velocity y from jacobian','relative task1 linear velocity z from jacobian',...
     'relative foothold2 linear velocity x from vrep','relative foothold2 linear velocity y from API','relative foothold2 linear velocity z from API',...
      'foothold2 to reference frame velocity x approximation','foothold2 to reference frame velocity y approximation','foothold2 to reference frame velocity z approximation');

figure(15)
plot(T,linear_velocity_r2_reference_vrep(1,:))
hold on;
plot(T,linear_velocity_r2_reference_vrep(2,:))
hold on;
plot(T,linear_velocity_r2_reference_vrep(3,:))
hold on;
plot(T,linear_velocity_r2_jacobian(1,:),':')
hold on;
plot(T,linear_velocity_r2_jacobian(2,:),':')
hold on;
plot(T,linear_velocity_r2_jacobian(3,:),':')
hold on;
plot(T,linear_velocity_r2_reference_vrep_API(1,:),'--')
hold on;
plot(T,linear_velocity_r2_reference_vrep_API(2,:),'--')
hold on;
plot(T,linear_velocity_r2_reference_vrep_API(3,:),'--')
hold on;
title('relative task2 linear velocity')
xlabel('Time/s') 
ylabel('relative task2 linear velocity from vrep and jacobian matlab') 
legend('relative task2 linear velocity x reference','relative task2 linear velocity y reference','relative task2 linear velocity z reference',...
    'relative task2 linear velocity x from jacobian','relative task2 linear velocity y from jacobian','relative task2 linear velocity z from jacobian',...
    'foothold3 linear velocity x from vrep','foothold3 linear velocity y from API','foothold3 linear velocity z from API');

figure(16)
plot(T,linear_velocity_r3_reference_vrep(1,:))
hold on;
plot(T,linear_velocity_r3_reference_vrep(2,:))
hold on;
plot(T,linear_velocity_r3_reference_vrep(3,:))
hold on;
plot(T,linear_velocity_r3_jacobian(1,:),':')
hold on;
plot(T,linear_velocity_r3_jacobian(2,:),':')
hold on;
plot(T,linear_velocity_r3_jacobian(3,:),':')
hold on;
plot(T,linear_velocity_r3_reference_vrep_API(1,:),'--')
hold on;
plot(T,linear_velocity_r3_reference_vrep_API(2,:),'--')
hold on;
plot(T,linear_velocity_r3_reference_vrep_API(3,:),'--')
hold on;
title('relative task3 linear velocity')
xlabel('Time/s') 
ylabel('relative task3 linear velocity from vrep and jacobian matlab') 
legend('relative task3 linear velocity x reference','relative task3 linear velocity y reference','relative task3 linear velocity z reference',...
    'relative task3 linear velocity x from jacobian','relative task3 linear velocity y from jacobian','relative task3 linear velocity z from jacobian',...
    'foothold4 linear velocity x from vrep','foothold4 linear velocity y from API','foothold4 linear velocity z from API');


figure(17)
plot(T,linear_velocity_r4_reference_vrep(1,:))
hold on;
plot(T,linear_velocity_r4_reference_vrep(2,:))
hold on;
plot(T,linear_velocity_r4_reference_vrep(3,:))
hold on;
plot(T,linear_velocity_r4_jacobian(1,:),':')
hold on;
plot(T,linear_velocity_r4_jacobian(2,:),':')
hold on;
plot(T,linear_velocity_r4_jacobian(3,:),':')
hold on;
plot(T,linear_velocity_r4_reference_vrep_API(1,:),'--')
hold on;
plot(T,linear_velocity_r4_reference_vrep_API(2,:),'--')
hold on;
plot(T,linear_velocity_r4_reference_vrep_API(3,:),'--')
hold on;
title('relative task4 linear velocity')
xlabel('Time/s') 
ylabel('relative task4 linear velocity from vrep and jacobian matlab') 
legend('relative task4 linear velocity x reference','relative task4 linear velocity y reference','relative task4 linear velocity z reference',...
    'relative task4 linear velocity x from jacobian','relative task4 linear velocity y from jacobian','relative task4 linear velocity z from jacobian',...
    'foothold5 linear velocity x from vrep','foothold5 linear velocity y from API','foothold5 linear velocity z from API');


figure(18)
plot(T,linear_velocity_r5_reference_vrep(1,:))
hold on;
plot(T,linear_velocity_r5_reference_vrep(2,:))
hold on;
plot(T,linear_velocity_r5_reference_vrep(3,:))
hold on;
plot(T,linear_velocity_r5_jacobian(1,:),':')
hold on;
plot(T,linear_velocity_r5_jacobian(2,:),':')
hold on;
plot(T,linear_velocity_r5_jacobian(3,:),':')
hold on;
plot(T,linear_velocity_r5_reference_vrep_API(1,:),'--')
hold on;
plot(T,linear_velocity_r5_reference_vrep_API(2,:),'--')
hold on;
plot(T,linear_velocity_r5_reference_vrep_API(3,:),'--')
hold on;
title('relative task5 linear velocity')
xlabel('Time/s') 
ylabel('relative task5 linear velocity from vrep and jacobian matlab') 
legend('relative task5 linear velocity x reference','relative task5 linear velocity y reference','relative task5 linear velocity z reference',...
    'relative task5 linear velocity x from jacobian','relative task5 linear velocity y from jacobian','relative task5 linear velocity z from jacobian',...
    'foothold6 linear velocity x from vrep','foothold6 linear velocity y from API','foothold6 linear velocity z from API');


figure(19)
plot(T,linear_velocity_foothold1_approximation(1,:))
hold on;
plot(T,linear_velocity_foothold1_approximation(2,:))
hold on;
plot(T,linear_velocity_foothold1_approximation(3,:))
hold on;
plot(T,linear_velocity_foothold1_vrep(1,:),'.')
hold on;
plot(T,linear_velocity_foothold1_vrep(2,:),'.')
hold on;
plot(T,linear_velocity_foothold1_vrep(3,:),'.')
hold on;
title('f1 linear velocity from approximation and vrep')
xlabel('Time/s') 
ylabel('f1 linear velocity from approximation and vrep') 
legend('f1 linear velocity x from approximation','f1 linear velocity y from approximation','f1 linear velocity z from approximation',...
    'f1 linear velocity x from vrep','f1 linear velocity y from vrep','f1 linear velocity z from vrep');

% figure(20)
% plot(T,angular_velocity_foothold1_approximation(1,:))
% hold on;
% plot(T,angular_velocity_foothold1_approximation(2,:))
% hold on;
% plot(T,angular_velocity_foothold1_approximation(3,:))
% hold on;
% plot(T,angular_velocity_foothold1_vrep(1,:),'.')
% hold on;
% plot(T,angular_velocity_foothold1_vrep(2,:),'.')
% hold on;
% plot(T,angular_velocity_foothold1_vrep(3,:),'.')
% hold on;
% title('f1 angular velocity from approximation and vrep')
% xlabel('Time/s') 
% ylabel('f1 angular velocity from approximation and vrep') 
% legend('f1 angular velocity x from approximation','f1 angular velocity y from approximation','f1 angular velocity z from approximation',...
%     'f1 angular velocity x from vrep','f1 angular velocity y from vrep','f1 angular velocity z from vrep');
% 
figure(21)
plot(T,vec_linear_velocity_body_jacobian(1,:))
hold on;
plot(T,vec_linear_velocity_body_jacobian(2,:))
hold on;
plot(T,vec_linear_velocity_body_jacobian(3,:))
hold on;
plot(T,vec_linear_velocity_body_vrep(1,:),'.')
hold on;
plot(T,vec_linear_velocity_body_vrep(2,:),'.')
hold on;
plot(T,vec_linear_velocity_body_vrep(3,:),'.')
hold on;
plot(T,vec_linear_velocity_body_approximation(1,:),'--')
hold on;
plot(T,vec_linear_velocity_body_approximation(2,:),'--')
hold on;
plot(T,vec_linear_velocity_body_approximation(3,:),'--')
hold on;
title('base linear velocity from jacobian and vrep and approximation')
xlabel('Time/s') 
ylabel('base linear velocity from jacobian and vrep and approximation') 
legend('base linear velocity x from jacobian','base linear velocity y from jacobian','base linear velocity z from jacobian',...
    'base linear velocity x from vrep','base linear velocity y from vrep','base linear velocity z from vrep',...
    'base linear velocity x from approximation','base linear velocity y from approximation','base linear velocity z from approximation');

figure(22)
plot(T,vec_angular_velocity_body_jacobian(1,:))
hold on;
plot(T,vec_angular_velocity_body_jacobian(2,:))
hold on;
plot(T,vec_angular_velocity_body_jacobian(3,:))
hold on;
plot(T,vec_angular_velocity_body_vrep(1,:),'.')
hold on;
plot(T,vec_angular_velocity_body_vrep(2,:),'.')
hold on;
plot(T,vec_angular_velocity_body_vrep(3,:),'.')
hold on;
plot(T,vec_angular_velocity_body_approximation(1,:),'--')
hold on;
plot(T,vec_angular_velocity_body_approximation(2,:),'--')
hold on;
plot(T,vec_angular_velocity_body_approximation(3,:),'--')
hold on;
title('base angular velocity from jacobian and vrep and approximation')
xlabel('Time/s') 
ylabel('base angular velocity from jacobian and vrep and approximation') 
legend('base angular velocity x from jacobian','base angular velocity y from jacobian','base angular velocity z from jacobian',...
    'base angular velocity x from vrep','base angular velocity y from vrep','base angular velocity z from vrep',...
    'base angular velocity x from approximation','base angular velocity y from approximation','base angular velocity z from approximation');

figure(23)
plot(T,angular_velocity_r1_jacobian(1,:),':')
hold on;
plot(T,angular_velocity_r1_jacobian(2,:),':')
hold on;
plot(T,angular_velocity_r1_jacobian(3,:),':')
hold on;
plot(T,angular_velocity_r1_reference_vrep_API(1,:),'--')
hold on;
plot(T,angular_velocity_r1_reference_vrep_API(2,:),'--')
hold on;
plot(T,angular_velocity_r1_reference_vrep_API(3,:),'--')
hold on;
title('relative task1 angular velocity')
xlabel('Time/s') 
ylabel('relative task1 angular velocity from vrep and jacobian matlab') 
legend('relative task1 angular velocity x from jacobian','relative task1 angular velocity y from jacobian','relative task1 angular velocity z from jacobian',...
    'foothold2 angular velocity x from vrep','foothold2 angular velocity y from API','foothold2 angular velocity z from API');

figure(24)
plot(T,angular_velocity_r2_jacobian(1,:),':')
hold on;
plot(T,angular_velocity_r2_jacobian(2,:),':')
hold on;
plot(T,angular_velocity_r2_jacobian(3,:),':')
hold on;
plot(T,angular_velocity_r2_reference_vrep_API(1,:),'--')
hold on;
plot(T,angular_velocity_r2_reference_vrep_API(2,:),'--')
hold on;
plot(T,angular_velocity_r2_reference_vrep_API(3,:),'--')
hold on;
title('relative task2 angular velocity')
xlabel('Time/s') 
ylabel('relative task2 angular velocity from vrep and jacobian matlab') 
legend('relative task2 angular velocity x from jacobian','relative task2 angular velocity y from jacobian','relative task2 angular velocity z from jacobian',...
    'foothold3 angular velocity x from vrep','foothold3 angular velocity y from API','foothold3 angular velocity z from API');

figure(25)
plot(T,angular_velocity_r3_jacobian(1,:),':')
hold on;
plot(T,angular_velocity_r3_jacobian(2,:),':')
hold on;
plot(T,angular_velocity_r3_jacobian(3,:),':')
hold on;
plot(T,angular_velocity_r3_reference_vrep_API(1,:),'--')
hold on;
plot(T,angular_velocity_r3_reference_vrep_API(2,:),'--')
hold on;
plot(T,angular_velocity_r3_reference_vrep_API(3,:),'--')
hold on;
title('relative task3 angular velocity')
xlabel('Time/s') 
ylabel('relative task3 angular velocity from vrep and jacobian matlab') 
legend('relative task3 angular velocity x from jacobian','relative task3 angular velocity y from jacobian','relative task3 angular velocity z from jacobian',...
    'foothold4 angular velocity x from vrep','foothold4 angular velocity y from API','foothold4 angular velocity z from API');

figure(26)
plot(T,angular_velocity_r4_jacobian(1,:),':')
hold on;
plot(T,angular_velocity_r4_jacobian(2,:),':')
hold on;
plot(T,angular_velocity_r4_jacobian(3,:),':')
hold on;
plot(T,angular_velocity_r4_reference_vrep_API(1,:),'--')
hold on;
plot(T,angular_velocity_r4_reference_vrep_API(2,:),'--')
hold on;
plot(T,angular_velocity_r4_reference_vrep_API(3,:),'--')
hold on;
title('relative task4 angular velocity')
xlabel('Time/s') 
ylabel('relative task4 angular velocity from vrep and jacobian matlab') 
legend('relative task4 angular velocity x from jacobian','relative task4 angular velocity y from jacobian','relative task4 angular velocity z from jacobian',...
    'foothold5 angular velocity x from vrep','foothold5 angular velocity y from API','foothold5 angular velocity z from API');

figure(27)
plot(T,angular_velocity_r5_jacobian(1,:),':')
hold on;
plot(T,angular_velocity_r5_jacobian(2,:),':')
hold on;
plot(T,angular_velocity_r5_jacobian(3,:),':')
hold on;
plot(T,angular_velocity_r5_reference_vrep_API(1,:),'--')
hold on;
plot(T,angular_velocity_r5_reference_vrep_API(2,:),'--')
hold on;
plot(T,angular_velocity_r5_reference_vrep_API(3,:),'--')
hold on;
title('relative task5 angular velocity')
xlabel('Time/s') 
ylabel('relative task5 angular velocity from vrep and jacobian matlab') 
legend('relative task5 angular velocity x from jacobian','relative task5 angular velocity y from jacobian','relative task5 angular velocity z from jacobian',...
    'foothold6 angular velocity x from vrep','foothold6 angular velocity y from API','foothold6 angular velocity z from API');



% figure(23)
% plot(T,vec_x_ref2base_dot_approximation(1,:))
% hold on
% plot(T,vec_x_ref2base_dot_jacobian(1,:),'--')
% 
% figure(24)
% plot(T,vec_x_ref2base_dot_approximation(2,:))
% hold on
% plot(T,vec_x_ref2base_dot_jacobian(2,:),'--')
% 
% figure(25)
% plot(T,vec_x_ref2base_dot_approximation(3,:))
% hold on
% plot(T,vec_x_ref2base_dot_jacobian(3,:),'--')
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

function trans = get_relative_translation_vector (footiplus1,footi)
footholdiplus1_to_footholdi = footiplus1' * footi;

footholdiplus1_to_footholdi_trans4 = vec4(footholdiplus1_to_footholdi.translation);
footholdiplus1_to_footholdi_trans = footholdiplus1_to_footholdi_trans4(2:4);
trans = footholdiplus1_to_footholdi_trans';


end

function [xi,angular_velocity, linear_velocity] = compute_velocity_from_xdot(xdot,current_pose)
xi = 2*xdot *current_pose';
angular_velocity_DQ = xi.P;
angular_velocity = vec3(angular_velocity_DQ);
p = current_pose.translation;
linear_velocity_DQ = xi.D-cross(p,angular_velocity);
linear_velocity = vec3(linear_velocity_DQ);
end