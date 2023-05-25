% Abstract class that defines a control law based on quadratic programming.
%
% Although there are inumerous controllers based on quadratic programming,
% this class is suitable for those whose objective function is based on
% task-space variables, such as the robot Jacobian and the task-space
% error.
%
% DQ_TaskspaceQuadraticProgrammingController Methods:
%   compute_objective_function_symmetric_matrix - (Abstract) Compute the matrix H used in the objective function qdot'*H*qdot + f'*qdot.
%   compute_objective_function_linear_component - (Abstract) Compute the vector f used in the objective function qdot'*H*qdot + f'*qdot.
%   compute_setpoint_control_signal - Based on the task setpoint, compute the control signal.
%   compute_tracking_control_signal - Based on the task trajectory, use the feedforward to compute the control signal.
% See also DQ_KinematicController, DQ_ClassicQPController.

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

classdef DQ_TaskspaceQuadraticProgrammingController_Hexapod < DQ_KinematicConstrainedController_Hexapod
    properties (Access = protected)
        solver;
    end
    
    methods (Abstract)
        compute_objective_function_symmetric_matrix(controller, J, task_error);
        compute_objective_function_linear_component(controller, J, task_error);
    end
    
    methods
        function controller = DQ_TaskspaceQuadraticProgrammingController_Hexapod(robot, solver)
            controller = controller@DQ_KinematicConstrainedController_Hexapod(robot);
            controller. solver = solver;
        end
        
        function [u,norm_task_error,norm_jacobian,vec_base_dot] = compute_setpoint_control_signal(controller, q, task_reference, footTip0_velocity)
            % Based on the task reference, compute the control signal
            if controller.is_set()
                % get the task variable according to the control objective
                task_variable = controller.get_task_variable(q);
                % get the Jacobian according to the control objective
                J_wb = controller.get_jacobian(q);
                %                 J = J_wb;
                J_consu = J_wb(:,9:26);
                J_consb = J_wb(:,1:8);
                
                % calculate the Euclidean error
                %                 task_error = task_variable - task_reference;
                
                task_initial_variable = [];
                task_initial_reference = [];
                if controller.control_objective == "HexapodTranslationTask"
                    task_initial_reference = [task_initial_reference; vec8(task_reference(1))];
                    task_initial_variable = task_variable;
                    for i = 2:6
                        a = task_reference(i).translation;
                        task_initial_reference = [task_initial_reference; vec4(a)];
                    end
                    task_error = task_initial_variable - task_initial_reference;
                elseif controller.control_objective == "HexapodTask"
%                     task_initial_reference = [task_initial_reference; vec8(task_reference(1))];
                    for i = 1:6
                        task_initial_variable = [task_initial_variable; vec8(task_variable(i))];
                        task_initial_reference = [task_initial_reference; vec8(task_reference(i))];
                    end
                    task_error = task_initial_variable - task_initial_reference;
                end
                a = J_wb * q;
                
                %                 task_error = zeros(48,1);
                %                 for i = 1:6
                %                     x_current = task_variable(i);
                %                     x_d = task_reference(i);
                % %                     norm_1 = norm(x_current' * x_d-1);
                % %                     norm_2 =  norm(x_current' * x_d+1);
                % %                     if norm_1 < norm_2
                % %                         task_error_dq = x_current' * x_d-1;
                % %                     else
                % %                         task_error_dq = x_current' * x_d+1
                % %                     end
                %                     task_error_dq = x_current' * x_d-1;
                %                     if i == 5
                %                         task_error(8*i-7:8*i) = vec8(task_error_dq);
                %                     else
                %                         task_error(8*i-7:8*i) = -vec8(task_error_dq);
                %                     end
                %                 end
                norm_task_error = norm(task_error+a);
                norm_task_error_2 = norm(task_error)
                norm_jacobian = norm(a);
                
                
                % calculate the parameters that quadprog use to solve the
                % quadratic problem min 0.5 * norm(J*u+gain*task_error)^2 + 0.5*norm(u)^2
                A = controller.inequality_constraint_matrix;
                b = controller.inequality_constraint_vector;
                Aeq = controller.equality_constraint_matrix;
                beq = controller.equality_constraint_vector;
                
                % compute the quadratic component of the objective function
                H = controller.compute_objective_function_symmetric_matrix_hexapod(J_consu, task_error);
                
                % compute the linear component of the objective function
                %                 f = controller.compute_objective_function_linear_component(J, task_error);
                f = controller.compute_objective_function_linear_component_hexapod(J_consu, task_error,J_consb,footTip0_velocity);
                
                u = controller.solver.solve_quadratic_program(H,f,A,b,Aeq,beq)
                
                % verify if the closed-loop system has reached a stable
                % region and update the appropriate flags accordingly.
                Conditions_for_exit = norm(controller.last_error_signal - task_error)
                controller.verify_stability(task_error);
                
                % Store the values of the last error signal and last
                % control signal
                controller.last_control_signal = u;
                controller.last_error_signal = task_error;
                
                %                 a = J_consu*u;
                %                 norm_task_error = norm(a);
                
                vec_absolute = zeros(11,1);
                vec_absolute(1:8) = vec8(footTip0_velocity);
                vec_absolute(9:11) = u(1:3);
                vec_base_dot = J_wb(1:8,1:11)* vec_absolute;
            end
        end
        
        function u = compute_tracking_control_signal(controller, q, ...
                task_reference, feedforward)
            if controller.is_set()
                % get the task variable according to the control objective
                task_variable = controller.get_task_variable(q);
                % get the Jacobian according to the control objective
                J = controller.get_jacobian(q);
                
                % calculate the Euclidean error
                task_error = task_variable - task_reference;
                
                % calculate the parameters that quadprog use to solve the
                % quadratic problem min 0.5 * norm(J*u+gain*task_error)^2 + 0.5*norm(u)^2
                A = controller.inequality_constraint_matrix;
                b = controller.inequality_constraint_vector;
                Aeq = controller.equality_constraint_matrix;
                beq = controller.equality_constraint_vector;
                
                % compute the quadratic component of the objective function
                H = controller.compute_objective_function_symmetric_matrix(J,...
                    task_error - (1/controller.gain)*feedforward);
                
                % compute the linear component of the objective function
                f = controller.compute_objective_function_linear_component(J,...
                    task_error - (1/controller.gain)*feedforward);
                
                u = controller.solver.solve_quadratic_program(H,f,A,b,Aeq,beq);
                
                % verify if the closed-loop system has reached a stable
                % region and update the appropriate flags accordingly.
                controller.verify_stability(task_error);
                
                % Store the values of the last error signal and last
                % control signal
                controller.last_control_signal = u;
                controller.last_error_signal = task_error;
            end
        end
        
    end
end