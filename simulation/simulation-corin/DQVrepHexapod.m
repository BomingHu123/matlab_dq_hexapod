classdef DQVrepHexapod < DQ_VrepRobot
    properties
        joint_names;
        base_frame_name;
    end
    
    methods
        function obj = DQVrepHexapod(robot_name,vrep_interface)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.robot_name = robot_name;
            obj.vrep_interface = vrep_interface;
            
            %This might be useful to every subclass
            splited_name = strsplit(robot_name,'#');
            robot_label = splited_name{1};
            if ~strcmp(robot_label,'hexapod')
%                 error('Expected Hexapod')
            end
            if length(splited_name) > 1
                robot_index = splited_name{2};
            else
                robot_index = '';
            end
            
            %Initialize joint names and base frame
            obj.joint_names = {};
            for i=1:18
                current_joint_name = {'/',robot_name,'/',robot_label,'_joint',int2str(i),robot_index};
%                 current_joint_name = {'/hexapod/',robot_label,'_joint',int2str(i),robot_index};
                obj.joint_names{i} = strjoin(current_joint_name,'');
            end
            base_frame_name = {'/',robot_name,'/','body'};
            obj.base_frame_name = strjoin(base_frame_name,'');
        end
        
        function send_q_to_vrep(obj,q)
            obj.vrep_interface.set_joint_positions(obj.joint_names,q);
        end
        
        function send_target_joint_to_vrep(obj,q)
            %             a = q(9:26);
            %             obj.vrep_interface.set_joint_positions(obj.joint_names,q);
            %               obj.vrep_interface.set_joint_positions(obj.joint_names,q)
            obj.vrep_interface.set_joint_target_positions(obj.joint_names,q);
%             a = DQ(q(1:8));
%             obj.vrep_interface.set_object_pose('/hexapod', a);
        end
        
        function q = get_q_from_vrep(obj)
            %             q = obj.vrep_interface.get_joint_positions(obj.joint_names);
            joint_q = obj.vrep_interface.get_joint_positions(obj.joint_names);
            dq_base = obj.vrep_interface.get_object_pose(obj.base_frame_name);
            base = (vec8(dq_base));
            q = [base;joint_q];
        end
        
        function q = get_object_pose_from_vrep(obj)
            q = obj.vrep_interface.get_object_pose('Hexapod_joint3','Hexapod_joint1',-1);
            while q == 0
                q = obj.vrep_interface.get_object_pose('Hexapod_joint3','Hexapod_joint1',-1);
            end
        end
        
        function q = get_object_pose_from_vrep2(obj)
            q = obj.vrep_interface.get_object_pose('Hexapod_joint2','Hexapod_joint1',-1);
            while q == 0
                q = obj.vrep_interface.get_object_pose('Hexapod_joint2','Hexapod_joint1',-1);
            end
        end
        
        function q = get_object_pose_from_vrep3(obj)
            q = obj.vrep_interface.get_object_pose('Hexapod_joint3','Hexapod_joint2',-1);
            while q == 0
                q = obj.vrep_interface.get_object_pose('Hexapod_joint3','Hexapod_joint2',-1);
            end
        end
        
        function q = get_object_pose_from_vrep_world3(obj)
            q = obj.vrep_interface.get_object_pose('Hexapod_joint3');
        end
        
        function q = get_object_pose_from_vrep_world1(obj)
            q = obj.vrep_interface.get_object_pose('Hexapod_joint1');
        end
        
        
        function q = get_foothold1_pose_from_vrep(obj)
            q = obj.vrep_interface.get_object_pose('/hexapod/footTip0');
        end
        
        function q = get_foothold2_pose_from_vrep(obj)
            q = obj.vrep_interface.get_object_pose('/hexapod/footTip1');
        end
        
        function q = get_foothold3_pose_from_vrep(obj)
            q = obj.vrep_interface.get_object_pose('/hexapod/footTip2');
        end
        
        function q = get_foothold4_pose_from_vrep(obj)
            q = obj.vrep_interface.get_object_pose('/hexapod/footTip3');
        end
        
        function q = get_foothold5_pose_from_vrep(obj)
            q = obj.vrep_interface.get_object_pose('/hexapod/footTip4');
        end
        
        function q = get_foothold6_pose_from_vrep(obj)
            q = obj.vrep_interface.get_object_pose('/hexapod/footTip5');
        end
        
        function q = get_body_pose_from_vrep(obj)
            q = obj.vrep_interface.get_object_pose('/hexapod/body');
        end
        
        function v = get_body_velocity_from_vrep(obj)
           v = obj.vrep_interface.vrep.simGetObjectVelocity('/hexapod');
        end
        
        
        
        function q = get_joint_from_vrep(obj)
            %             q = obj.vrep_interface.get_joint_positions(obj.joint_names);
            q = obj.vrep_interface.get_joint_positions(obj.joint_names);
            %             dq_base = obj.vrep_interface.get_object_pose(obj.base_frame_name);
            %             base = (vec8(dq_base))';
            %             q = [base,joint_q];
        end
        
        function q = get_reference_feethold1_from_vrep(obj)
            q = obj.vrep_interface.get_object_pose('/hexapod/footTip0',-1,-1);
        end
        
        function robot = kinematics(obj)
            % Create a new corinhexapod Create Robot
            
            % The parameters below are given in meters
            include_namespace_dq
            
            
            %             arm_DH_theta = [0,-pi/6,0];
            
            arm_DH_theta = [0,0,0];
            arm_DH_d = [-0.0112, 0, 0];
            arm_DH_a = [0.0503, 0.0724, 0];
            arm_DH_alpha = [-pi/2, 0,0];
            
            arm_DH_matrix = [arm_DH_theta; arm_DH_d; arm_DH_a;arm_DH_alpha];
            leg1 = DQ_SerialManipulator(arm_DH_matrix ,'standard');
            leg2 = DQ_SerialManipulator(arm_DH_matrix ,'standard');
            %             leg = DQ_MultilegRobot_Leg(arm_DH_matrix ,'mdh');
            
            include_namespace_dq
            r =cos(pi/4) + i_*sin(pi/4);
            p = 0.1159904897*i_ - 0.01105138659*j_ + 0*k_;
            end_effector1 = r + E_*0.5*p*r;    
            leg1.set_effector(end_effector1);
            
            r =-cos(-pi/4) + i_*sin(-pi/4);
            p = 0.1159904897*i_ - 0.01105138659*j_ + 0*k_;
            end_effector2 = r + E_*0.5*p*r;    
            leg2.set_effector(end_effector2);
            
            %             base = DQ_HolonomicBase();
            base = DQ_HexapodBase();
            robot = DQ_MultiLeggedRobotBody(base);
            
            r =cos(0) + k_*sin(0);
            p = 0.08*i_ + 0*j_ + 0.0031*k_;
            bias_crotch1 = r + E_*0.5*p*r;
            robot.add(leg1,bias_crotch1);
            
            r =cos(pi/6) + k_*sin(pi/6);
            p = 0.04*i_ + 0.0693*j_ + 0.0031*k_;
            bias_crotch2 = r + E_*0.5*p*r;
            robot.add(leg1,bias_crotch2);
            
            r =cos(2*pi/6) + k_*sin(2*pi/6);
            p = -0.04*i_ + 0.0693*j_ + 0.0031*k_;
            bias_crotch3 = r + E_*0.5*p*r;
            robot.add(leg2,bias_crotch3);
            
            r =cos(pi/2) + k_*sin(pi/2);
            p = -0.08*i_ - 0*j_ + 0.0031*k_;
            bias_crotch4 = r + E_*0.5*p*r;
            robot.add(leg2,bias_crotch4);
            
            r =cos(-2*pi/6) + k_*sin(-2*pi/6);
            p = -0.04*i_ - 0.0693*j_ + 0.0031*k_;
            r = -r;
            bias_crotch5 = r + E_*0.5*p*r;
            robot.add(leg1,bias_crotch5);
            
            r =cos(-pi/6) + k_*sin(-pi/6);
            p = 0.04*i_ - 0.0693*j_ + 0.0031*k_;
            r = -r;
            bias_crotch6 = r + E_*0.5*p*r;
            robot.add(leg1,bias_crotch6);
            
            %             robot.set_reference_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            robot.set_reference_frame(DQ(1));
            robot.set_base_frame(obj.vrep_interface.get_object_pose(obj.base_frame_name));
            %             robot.set_base_frame(DQ(1));
            
            
        end
    end
end