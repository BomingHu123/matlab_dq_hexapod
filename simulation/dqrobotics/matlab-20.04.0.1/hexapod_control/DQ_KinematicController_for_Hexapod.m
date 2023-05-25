classdef DQ_KinematicController_for_Hexapod < DQ_KinematicController
    methods
        function controller = DQ_KinematicController_for_Hexapod(robot)
            controller = controller@DQ_KinematicController(robot);
        end
        
        function J = get_jacobian(controller, q)
            x_pose = controller.robot.fkm(q);
%             J_pose = controller.robot.pose_jacobian(q);
            %             primitive = controller.attached_primitive;
            
            primitive = controller.attached_primitive;
%             J = J_pose;
            switch controller.control_objective
                case ControlObjective.HexapodTask
                    J_pose = controller.robot.pose_jacobian(q);
                    J = J_pose;
                case ControlObjective.HexapodAbs
                    J_pose = controller.robot.pose_jacobian(q);
                    J = J_pose(1:8,1:8);
                case ControlObjective.HexapodRel
                    J_pose = controller.robot.pose_jacobian(q);
                    J = J_pose(:,9:26);
                case ControlObjective.HexapodTranslationTask
                    J_pose = controller.robot.pose_jacobian_relative_translation(q);
                    J = J_pose;
            end
        end
        
        function task_variable = get_task_variable(controller, q)
            x_pose = controller.robot.fkm(q);
            primitive = controller.attached_primitive;
            
            switch controller.control_objective
                case ControlObjective.HexapodTask
                    task_variable = x_pose;
%                     task_variable = [];
%                     for i = 1 : 6
%                         task = vec8(x_pose(i));
%                         task_variable = [task_variable; task];
%                     end

                    
                case ControlObjective.HexapodAbs
                    task_variable = [];
                    for i = 1
                        task = vec8(x_pose(i));
                        task_variable = [task_variable; task];
                    end
                    
                case ControlObjective.HexapodTask
                    task_variable = [];
                    for i = 2 : 6
                        task = vec8(x_pose(i));
                        task_variable = [task_variable; task];
                    end
                    
                case ControlObjective.HexapodTranslationTask
                   task_variable = [];
                   task_1 = vec8(x_pose(1));
                   task_variable = [task_variable;task_1];
                   for i = 2:6
                       relative_translation = x_pose(i).translation;
                       task = vec4(relative_translation);
                       task_variable = [task_variable; task];
                   end
            end
        end
        
        function set_control_objective(controller,control_objective)
            if isa(control_objective, 'ControlObjective')
                controller.system_reached_stable_region_ = false;
                controller.control_objective = control_objective;
                
                switch control_objective
                    case {ControlObjective.HexapodTask}
                        controller.last_error_signal = zeros(48,1);
                    case {ControlObjective.HexapodAbs}
                        controller.last_error_signal = zeros(8,1);
                    case {ControlObjective.HexapodRel}
                        controller.last_error_signal = zeros(40,1);
                    case {ControlObjective.HexapodTranslationTask}
                        controller.last_error_signal = zeros(28,1);
                end
                
            else
                error(['Only objectives enumerated in ControlObjective are '
                    'allowed']);
            end
        end
    end
end