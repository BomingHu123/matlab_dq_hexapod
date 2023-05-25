% Abstract class that defines a control law based on quadratic programming.
%
% DQ_ClassicQPController Methods:
%   compute_objective_function_symmetric_matrix - compute the matrix H used in the objective function qdot'*H*qdot + f'*qdot
%   compute_objective_function_linear_component - compute the vector f used in the objective function qdot'*H*qdot + f'*qdot
%
% For more methods and properties, see also DQ_KinematicController.

% (C) Copyright 2011-2019 DQ Robotics Developers
%
% This file is part of DQ Robotics.
%
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as
%     published by the Free Software Foundation, either version 3 of the
%     License, or (at your option) any later version.
%
%     DQ Robotics is distributed in the hope that it will be useful, but
%     WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%     Lesser General Public License for more details.
%
%     You should have received a copy of the GNU Lesser General Public
%     License along with DQ Robotics.  If not, see
%     <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br

classdef DQ_ClassicQPController_Hexapod < DQ_TaskspaceQuadraticProgrammingController_Hexapod
    methods
        function controller = DQ_ClassicQPController_Hexapod(robot,solver)
            controller = controller@DQ_TaskspaceQuadraticProgrammingController_Hexapod(robot,solver);
            % Default damping
            controller.damping = 1e-3;
        end
        
        function H = compute_objective_function_symmetric_matrix(controller, ...
                J, ~)
            % Compute the matrix H used in the objective function qdot'*H*qdot + f'*qdot
            
            H = J'*J + controller.damping*eye(controller.robot.get_dim_configuration_space());
        end
        
        function f = compute_objective_function_linear_component(controller,...
                J, task_error)
            % Compute the vector f used in the objective function qdot'*H*qdot + f'*qdot
            f = J'*controller.gain*task_error;
        end
        
        function H = compute_objective_function_symmetric_matrix_hexapod(controller, ...
                J, ~)
            % Compute the matrix H used in the objective function qdot'*H*qdot + f'*qdot
            H = J'*J + controller.damping*eye(controller.robot.get_dim_configuration_space());
        end
        
        function f = compute_objective_function_linear_component_hexapod(controller,...
                J, task_error,Ja,velocity_footTip0)
            % Compute the vector f used in the objective function qdot'*H*qdot + f'*qdot
%             gain = eye(28);
%             for i = 1:8
%                 gain(i,i) = 1;
%             end
            f = J'*((Ja * vec8(velocity_footTip0)) + controller.gain *task_error);
%             f = J'*controller.gain*task_error;
        end
    end
end