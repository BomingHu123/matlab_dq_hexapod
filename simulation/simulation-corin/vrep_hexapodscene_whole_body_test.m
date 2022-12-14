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

Foothold1_to_foothold2_trans = [];
Foothold2_to_foothold3_trans = [];
Foothold3_to_foothold4_trans = [];
Foothold4_to_foothold5_trans = [];
Foothold5_to_foothold6_trans = [];
Body_copp_trans = [];

Foothold1_to_foothold2_matlab_trans = [];
Foothold2_to_foothold3_matlab_trans = [];
Foothold3_to_foothold4_matlab_trans = [];
Foothold4_to_foothold5_matlab_trans = [];
Foothold5_to_foothold6_matlab_trans = [];
Body_matlab_trans = [];

Norm_of_error = [];

    a = 1;
%% Control loop
for t=0:sampling_time:total_time
    %% get information from CoppeliaSim
    pause(0.05);
    
    foothold1 = corin_hexapod.get_foothold1_pose_from_vrep();
    foothold2 = corin_hexapod.get_foothold2_pose_from_vrep();
    foothold3 = corin_hexapod.get_foothold3_pose_from_vrep();
    foothold4 = corin_hexapod.get_foothold4_pose_from_vrep();
    foothold5 = corin_hexapod.get_foothold5_pose_from_vrep();
    foothold6 = corin_hexapod.get_foothold6_pose_from_vrep();
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
    
    Foothold1_to_foothold2_trans = [Foothold1_to_foothold2_trans; foothold1_to_foothold2_trans'];
    Foothold2_to_foothold3_trans = [Foothold2_to_foothold3_trans; foothold2_to_foothold3_trans'];
    Foothold3_to_foothold4_trans = [Foothold3_to_foothold4_trans; foothold3_to_foothold4_trans'];
    Foothold4_to_foothold5_trans = [Foothold4_to_foothold5_trans; foothold4_to_foothold5_trans'];
    Foothold5_to_foothold6_trans = [Foothold5_to_foothold6_trans; foothold5_to_foothold6_trans'];
    Body_copp_trans = [Body_copp_trans; body_copp_trans'];
    
    %% get information from Matlab
    foothold1 = corin_hexapod.get_reference_joint1_from_vrep;
%     corin.set_reference_to_first_feethold(corin_q);
    corin.set_reference_to_first_feethold_vrep(foothold1);
    
    corin_q = corin_hexapod.get_q_from_vrep();
    
%     body_matlab = corin.fkm(corin_q,1);
    foothold1_matlab = corin.fkm(corin_q,2);
    foothold2_matlab = corin.fkm(corin_q,3);
    foothold3_matlab = corin.fkm(corin_q,4);
    foothold4_matlab = corin.fkm(corin_q,5);
    foothold5_matlab = corin.fkm(corin_q,6);
    foothold6_matlab = corin.fkm(corin_q,7);
    
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

    
    Body_matlab_trans = [Body_matlab_trans;body_matlab_trans'];
    Foothold1_to_foothold2_matlab_trans = [Foothold1_to_foothold2_matlab_trans;foothold1_to_foothold2_matlab_trans'];
    Foothold2_to_foothold3_matlab_trans = [Foothold2_to_foothold3_matlab_trans;foothold2_to_foothold3_matlab_trans'];
    Foothold3_to_foothold4_matlab_trans = [Foothold3_to_foothold4_matlab_trans;foothold3_to_foothold4_matlab_trans'];
    Foothold4_to_foothold5_matlab_trans = [Foothold4_to_foothold5_matlab_trans;foothold4_to_foothold5_matlab_trans'];
    Foothold5_to_foothold6_matlab_trans = [Foothold5_to_foothold6_matlab_trans;foothold5_to_foothold6_matlab_trans'];
    
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
    
    nor = norm(task_error);
    Norm_of_error = [Norm_of_error;nor];
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
    corin_hexapod.send_q_to_vrep(corin_q(9:26));
    
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
    
    
end


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





