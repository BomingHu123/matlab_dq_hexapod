classdef DQ_KinematicController_for_Hexapod < DQ_KinematicController
    methods
        function controller = DQ_KinematicController_for_Hexapod(robot)
            controller = controller@DQ_KinematicController(robot);
        end
        
        function J = get_jacobian(controller, q)
            x_pose = controller.robot.fkm(q);
            J_pose = controller.robot.pose_jacobian(q);
            %             primitive = controller.attached_primitive;
            
            primitive = controller.attached_primitive;
            J = J_pose;
            switch controller.control_objective
                case ControlObjective.HexapodTask
                    %                     A = zeros(26,26);
                    %                     for i = 9:26
                    %                         A(i,i) = 1;
                    %                     end
                    %                     J = J_pose * A;
                    %                     J = J(:,9:26);
                    J = J_pose;
                    % % %                 J = J_pose;
            end
        end
        
        function task_variable = get_task_variable(controller, q)
            x_pose = controller.robot.fkm(q);
            primitive = controller.attached_primitive;
            
            switch controller.control_objective
                case ControlObjective.HexapodTask
                    task_variable = [];
                    for i = 1 : 6
                        task = vec8(x_pose(i));
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
                end
                
            else
                error(['Only objectives enumerated in ControlObjective are '
                    'allowed']);
            end
        end
    end
end