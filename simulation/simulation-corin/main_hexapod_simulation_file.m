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
%     DQ Robotics is free software: you can redistribute it and/or modify
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

clear all;
close all;
clc;


% False if we don' want to show intermediate frames
simulation_parameters.show_frames = false;
% Total simulation time, in seconds.
simulation_parameters.total_time = 40000;

% The maximum radius that the manipulator will perform
simulation_parameters.dispz = 0.001;
% Occilation along the radius
simulation_parameters.wd = 0.005;
% Occilation around the rotation axix
simulation_parameters.wn = 0.001;

% vrep_hexapodscene_dqcontroller(simulation_parameters);
vrep_hexapodscene_dqcontroller_without_constraint(simulation_parameters);

%% single leg kinematic test
% [Foothold1_trans, Foothold2_trans, Foothold3_trans, Foothold4_trans, Foothold5_trans, Foothold6_trans,...
%     Foothold1_matlab_trans,Foothold2_matlab_trans,Foothold3_matlab_trans,Foothold4_matlab_trans,...
%     Foothold5_matlab_trans,Foothold6_matlab_trans,Body_copp_trans,Body_matlab_trans,Norm_of_error]= vrep_hexapodscene_single_leg_test(simulation_parameters)

%% whole body kinematic test
% [Foothold1_to_foothold2_trans, Foothold2_to_foothold3_trans, Foothold3_to_foothold4_trans, Foothold4_to_foothold5_trans, Foothold5_to_foothold6_trans,...
%     Foothold1_to_foothold2_matlab_trans,Foothold2_matlab_trans,Foothold3_matlab_trans,Foothold4_matlab_trans,...
%     Foothold5_matlab_trans,Body_copp_trans,Body_matlab_trans,Norm_of_error]= vrep_hexapodscene_whole_body_test(simulation_parameters);
%%
% [Foothold1_trans,Foothold1_matlab_trans,Foothold1_rot,Foothold1_matlab_rot]=vrep_hexapodscene_demo(simulation_parameters);

% plot3(Foothold1_trans(:,1),Foothold1_trans(:,2),Foothold1_trans(:,3),'*')
% hold on;
% plot3(Foothold1_matlab_trans(:,1),Foothold1_matlab_trans(:,2),Foothold1_matlab_trans(:,3),'r--o')
% legend('foothold4 from coppeliasim','foothold4 from matlab')

% A = (1:401);
% plot(A(:),Foothold1_rot(:,1))
% hold on 
% plot(A(:),Foothold1_matlab_rot(:,1))
% legend('x eular angle for foothold4 from coppeliasim','x eular angle for foothold4 from matlab')
% figure on
% plot(A(:),Foothold1_rot(:,2))
% hold on 
% plot(A(:),Foothold1_matlab_rot(:,2))
% legend('y eular angle for foothold4 from coppeliasim','y eular angle for foothold4 from matlab')
% figure on
% plot(A(:),Foothold1_rot(:,3))
% hold on 
% plot(A(:),Foothold1_matlab_rot(:,3))
% legend('z eular angle for foothold4 from coppeliasim','z eular angle for foothold4 from matlab')

%%
% [Foothold1_trans, Foothold2_trans, Foothold3_trans, Foothold4_trans, Foothold5_trans, Foothold6_trans,...
%     Foothold1_matlab_trans,Foothold2_matlab_trans,Foothold3_matlab_trans,Foothold4_matlab_trans,...
%     Foothold5_matlab_trans,Foothold6_matlab_trans,Body_copp_trans,Body_matlab_trans,Norm_of_error]= vrep_hexapodscene_dqcontroller2(simulation_parameters);
% 
% plot3(Foothold1_trans(:,1),Foothold1_trans(:,2),Foothold1_trans(:,3),'b')
% hold on;
% plot3(Foothold2_matlab_trans(:,1),Foothold2_matlab_trans(:,2),Foothold2_matlab_trans(:,3),'--')
% hold on;
% 
% plot3(Foothold2_trans(:,1),Foothold2_trans(:,2),Foothold2_trans(:,3),'r')
% hold on;
% plot3(Foothold2_matlab_trans(:,1),Foothold2_matlab_trans(:,2),Foothold2_matlab_trans(:,3),'--')
% hold on;
% 
% plot3(Foothold3_trans(:,1),Foothold3_trans(:,2),Foothold3_trans(:,3),'g')
% hold on;
% plot3(Foothold3_matlab_trans(:,1),Foothold3_matlab_trans(:,2),Foothold3_matlab_trans(:,3),'--')
% hold on;
% 
% plot3(Foothold4_trans(:,1),Foothold4_trans(:,2),Foothold4_trans(:,3),'k')
% hold on;
% plot3(Foothold4_matlab_trans(:,1),Foothold4_matlab_trans(:,2),Foothold4_matlab_trans(:,3),'--')
% hold on;
% 
% plot3(Foothold5_trans(:,1),Foothold5_trans(:,2),Foothold5_trans(:,3),'c')
% hold on;
% plot3(Foothold5_matlab_trans(:,1),Foothold5_matlab_trans(:,2),Foothold5_matlab_trans(:,3),'--')
% hold on;
% 
% plot3(Foothold6_trans(:,1),Foothold6_trans(:,2),Foothold6_trans(:,3),'m')
% hold on;
% plot3(Foothold6_matlab_trans(:,1),Foothold6_matlab_trans(:,2),Foothold6_matlab_trans(:,3),'--')
% hold on;
% 
% plot3(Body_copp_trans(:,1),Body_copp_trans(:,2),Body_copp_trans(:,3))
% hold on;
% plot3(Body_matlab_trans(:,1),Body_matlab_trans(:,2),Body_matlab_trans(:,3),'--')


% A = (1:length(Norm_of_error));
% plot(A(:)*0.05,Norm_of_error(:)); 
% ylabel('Norm of error') 
% xlabel('Time/s') 

