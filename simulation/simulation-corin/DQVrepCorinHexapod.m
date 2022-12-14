classdef DQVrepCorinHexapod < DQ_VrepRobot
    properties
        joint_names;
        base_frame_name;
    end
    
    methods
        function obj = DQVrepCorinHexapod(robot_name,vrep_interface)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
            %This might be useful to every subclass
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'Corin')
                error('Expected Corin')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            %Initialize joint names and base frame
            obj.joint_names = {};
            for i=1:18
                current_joint_name = {robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
            obj.base_frame_name = robot_name;
        end
        
        function send_q_to_vrep(obj,q)
            include_namespace_dq;
            pose = q(1) + q(2)*i_ + q(3)*j_ + q(4)*k_ + E_ * (q(5) + q(6)*i_ + q(7)*j_ + q(8)*k_);
            obj.vrep_interface.set_joint_positions(obj.joint_names,q);
            obj.vrep_interface.set_object_pose(obj.base_frame_name, pose);
        end
        
        function send_joint_to_vrep(obj,q)
            a = q(9:26);
            obj.vrep_interface.set_joint_positions(obj.joint_names,a);
        end
        
        function q = get_q_from_vrep(obj)
            %             q = obj.vrep_interface.get_joint_positions(obj.joint_names);
            joint_q = obj.vrep_interface.get_joint_positions(obj.joint_names);
            dq_base = obj.vrep_interface.get_object_pose(obj.base_frame_name);
            base = (vec8(dq_base))';
            q = [base,joint_q];
        end
        
        function q = get_joint_from_vrep(obj)
            %             q = obj.vrep_interface.get_joint_positions(obj.joint_names);
            q = obj.vrep_interface.get_joint_positions(obj.joint_names);
            %             dq_base = obj.vrep_interface.get_object_pose(obj.base_frame_name);
            %             base = (vec8(dq_base))';
            %             q = [base,joint_q];
        end
        
        function robot = kinematics(obj)
            % Create a new corinhexapod Create Robot
            
            % The parameters below are given in meters
            include_namespace_dq
            front2back = 0.115*2;
            left2right = 0.09*2;
            
            arm_DH_theta = [0,0,-pi/2];
            arm_DH_d = [0, 0, 0];
            arm_DH_a = [0.077, 0.15, 0.17];
            arm_DH_alpha = [pi/2, 0, 0];
            arm_DH_matrix = [arm_DH_theta; arm_DH_d; arm_DH_a;arm_DH_alpha];
            %             leg = DQ_SerialManipulator(arm_DH_matrix ,'standard');
            leg = DQ_MultilegRobot_Leg(arm_DH_matrix ,'standard');
            
            %             base = DQ_HolonomicBase();
            base = DQ_HexapodBase();
            robot = DQ_MultiLeggedRobotBody(base);
            
            r =cos(pi/2) + k_*sin(pi/2);
            p = -left2right/2*i_ + front2back/2*j_ + 0*k_;
            bias_left_fornt = r + E_*0.5*p*r;
            robot.add(leg,bias_left_fornt);
            
            r =cos(pi/2) + k_*sin(pi/2);
            p = -left2right/2*i_ + 0*j_ + 0*k_;
            bias_left_mid = r + E_*0.5*p*r;
            robot.add(leg,bias_left_mid);
            
            r =cos(pi/2) + k_*sin(pi/2);
            p = -left2right/2*i_ - front2back/2*j_ + 0*k_;
            bias_left_back = r + E_*0.5*p*r;
            robot.add(leg,bias_left_back);
            
            r =cos(0) + k_*sin(0);
            p = left2right/2*i_ - front2back/2*j_ + 0*k_;
            bias_right_back = r + E_*0.5*p*r;
            robot.add(leg,bias_right_back);
            
            r =cos(0) + k_*sin(0);
            p = left2right/2*i_ + 0*j_ + 0*k_;
            bias_right_mid = r + E_*0.5*p*r;
            robot.add(leg,bias_right_mid);
            
            r =cos(0) + k_*sin(0);
            p = left2right/2*i_ + front2back/2*j_ + 0*k_;
            bias_right_front = r + E_*0.5*p*r;
            robot.add(leg,bias_right_front);
            
            
        end
    end
end