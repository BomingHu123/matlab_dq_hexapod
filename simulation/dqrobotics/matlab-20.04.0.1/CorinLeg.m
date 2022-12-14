

classdef CorinLeg < DQ_Kinematics
    methods (Static)
        function ret = kinematics()
            %Standard D-H of KUKA-LWR
            Corin_DH_theta=[0, 0, 0, 0, 0, 0, 0];
            Corin_DH_d = [0.310, 0, 0.4, 0, 0.39, 0, 0];
            Corin_DH_a = [0, 0, 0, 0, 0, 0, 0];
            Corin_DH_alpha = [pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0];
            Corin_DH_type = repmat(DQ_SerialManipulatorDH.JOINT_ROTATIONAL,1,7);
            Corin_DH_matrix = [Corin_DH_theta;
                Corin_DH_d;
                Corin_DH_a;
                Corin_DH_alpha;
                Corin_DH_type];

            ret = DQ_SerialManipulatorDH(Corin_DH_matrix,'standard');
        end
    end
end
