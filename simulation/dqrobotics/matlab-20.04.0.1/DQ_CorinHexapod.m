% (C) Copyright 2011-2019 DQ Robotics Developers
% Build the Corin Hexapod Robot based on the DQ library
%
% Contributors to this file:
%     Boming Hu - boming.hu@postgrad.manchester.ac.uk

classdef DQ_CorinHexapod
    methods (Static)
        function robot = kinematics()
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


