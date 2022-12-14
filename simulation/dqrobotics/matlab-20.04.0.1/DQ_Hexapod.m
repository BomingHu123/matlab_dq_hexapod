% (C) Copyright 2011-2019 DQ Robotics Developers
% Build the Corin Hexapod Robot based on the DQ library
%
% Contributors to this file:
%     Boming Hu - boming.hu@postgrad.manchester.ac.uk

classdef DQ_Hexapod
    methods (Static)
        function robot = kinematics()
            % Create a new corinhexapod Create Robot
            
            % The parameters below are given in meters
            include_namespace_dq

            
            arm_DH_theta = [0,-pi/6,2 * pi/3];
%             arm_DH_theta = [0,0,0];
            arm_DH_d = [-0.0112, 0, 0];
            arm_DH_a = [0.0503, 0.0724, 0.116];
            arm_DH_alpha = [-pi/2, 0, 0];
            arm_DH_matrix = [arm_DH_theta; arm_DH_d; arm_DH_a;arm_DH_alpha];
            %             leg = DQ_SerialManipulator(arm_DH_matrix ,'standard');
            leg = DQ_MultilegRobot_Leg(arm_DH_matrix ,'standard');
            
            %             base = DQ_HolonomicBase();
            base = DQ_HexapodBase();
            robot = DQ_MultiLeggedRobotBody(base);
            
            r =cos(0) + k_*sin(0);
            p = 0.08*i_ + 0*j_ + 0*k_;
            bias_crotch1 = r + E_*0.5*p*r;
            robot.add(leg,bias_crotch1);
            
            r =cos(pi/6) + k_*sin(pi/6);
            p = 0.04*i_ + 0.0693*j_ + 0*k_;
            bias_crotch2 = r + E_*0.5*p*r;
            robot.add(leg,bias_crotch2);
            
            r =cos(2*pi/6) + k_*sin(2*pi/6);
            p = -0.04*i_ + 0.0693*j_ + 0*k_;
            bias_crotch3 = r + E_*0.5*p*r;
            robot.add(leg,bias_crotch3);
            
            r =cos(pi/2) + k_*sin(pi/2);
            p = -0.08*i_ - 0*j_ + 0*k_;
            bias_crotch4 = r + E_*0.5*p*r;
            robot.add(leg,bias_crotch4);
            
            r =cos(-2*pi/6) + k_*sin(-2*pi/6);
            p = -0.04*i_ - 0.0693*j_ + 0*k_;
            bias_crotch5 = r + E_*0.5*p*r;
            robot.add(leg,bias_crotch5);
            
            r =cos(-pi/6) + k_*sin(-pi/6);
            p = 0.04*i_ - 0.0693*j_ + 0*k_;
            bias_crotch5 = r + E_*0.5*p*r;
            robot.add(leg,bias_crotch5);
            

        end
    end
end


