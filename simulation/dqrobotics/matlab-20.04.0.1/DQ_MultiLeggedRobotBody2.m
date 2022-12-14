% DQ_MultiLeggedRobotBody Robots composed of multiple kinematic chains of
% multi-legged robot

% Usage: robot = DQ_MultiLeggedRobotBody(first_chain_element), where
% first_chain_element is an object of DQ_Kinematics or one of its
% subclasses.

% DQ_WholeBody Properties:
%       chain - Contains all elements in the serial kinematic chain.
%       frame_bias - DQ contain the bias of each leg base to the orginal frame
%       dim_configuration_space - Dimension of the each leg configuration
%                               space

% DQ_WholeBody Methods:
%       add - Adds a new element to main body kinematic chain with bias.
%       fkm - Returns the forward kinematic model of the body chain or the
%             forward kinematic model of the legs chain.
%       get_dim_configuration_space - Returns the dimension of the body or
%                                     each leg configuration space
%       pose_jacobian - Returns the body pose or feet of legs Jacobian.
%       raw_fkm - Analogous to FKM, but without considering base and
%                 end-effector changes.
%
% (C) Copyright 2011-2019 DQ Robotics Developers
% Contributors to this file:
%     Boming Hu - boming.hu@postgrad.manchester.ac.uk

classdef DQ_MultiLeggedRobotBody2 < DQ_Kinematics
    properties (Access = protected)
        chain;
        frame_bias;
        dim_configuration_space
        reference_to_first_feethold
    end
    methods
        function obj = DQ_MultiLeggedRobotBody(robot)
            include_namespace_dq;
            if ~isa(robot,'DQ_Kinematics')
                error(['The first argument must be a DQ_Kinematics '...
                    'object or one of its subclasses.']);
            end
            obj.chain{1} = robot;
            obj.dim_configuration_space{1} = 0;
            r =cos(0) + k_*sin(0);
            p = 0;
            xd = r + E_*0.5*p*r;
            obj.frame_bias{1} = xd;
% x = 0
        end
        
        function add(obj, new_chain,offset)
            if nargin > 3
                error('Invalid number of arguments');
            elseif nargin == 3
                len = length(obj.chain);
                obj.chain{len + 1} = new_chain;
                if isa(new_chain,'DQ_Kinematics')
                    obj.dim_configuration_space{len + 1} = new_chain.get_dim_configuration_space();
                elseif ~isa(new_chain,'DQ')
                    error(['Only DQ_Kinematics and DQ objects can be added to the'...
                        ' chain']);
                end
                if isa(offset, 'DQ')
                    obj.frame_bias{len + 1} = offset;
                else
                    error(['DQ objects can be added to the'...
                        ' bias']);
                end
            else
                len = length(obj.chain);
                obj.chain{len + 1} = new_chain;
                if isa(new_chain,'DQ_Kinematics')
                    obj.dim_configuration_space{len + 1} = new_chain.get_dim_configuration_space();
                elseif ~isa(new_chain,'DQ')
                    error(['Only DQ_Kinematics and DQ objects can be added to the'...
                        ' chain']);
                end
                if isa(offset, 'DQ')
                    obj.frame_bias{len + 1} = obj.frame_bias{1};
                else
                    error(['DQ objects can be added to the'...
                        ' bias']);
                end
            end
        end
        
        function x = fkm(obj,q,num_of_chain,ith)
            include_namespace_dq;
            % Returns the forward kinematic model of the whole-body chain.
            %
            % x = FKM(q) receives the configuration vector q of the whole
            % kinematic chain and returns the pose of the last frame.
            % x = FKM(q, num_of_chain) calculates the forward kinematics up to the num_of_chain
            % kinematic chain.
            % x = RAW_FKM(q, num_of_chain, ith) calculates the forward kinematics up to
            % the ith link of the ith kinematic chain.
            % FKM takes into account the reference frame.
            if nargin > 4
                error('Invalid number of arguments');
            elseif nargin == 4
                x = obj.reference_frame * obj.frame_bias{num_of_chain + 1} * raw_fkm(obj,q,num_of_chain,ith);
            elseif nargin == 3
                if num_of_chain == 1
                    first_cortch_to_first_feethold = raw_fkm_absolute(obj,q,1);
                    x = obj.reference_to_first_feethold * first_cortch_to_first_feethold' * obj.frame_bias{2}';
%                     x = obj.base_frame * obj.frame_bias{num_of_chain + 1} * raw_fkm_absolute(obj,q,num_of_chain);
                else
                    first_cortch_to_first_feethold = raw_fkm_absolute(obj,q,1);
                    base = obj.reference_to_first_feethold * first_cortch_to_first_feethold' * obj.frame_bias{2}';
                    x = base * obj.frame_bias{num_of_chain} * raw_fkm_absolute(obj,q,num_of_chain-1);
                end
            else
                %                 x_absoulute_pose = obj.reference_frame * obj.frame_bias{1} * raw_fkm(obj,q,0);
                first_cortch_to_first_feethold = raw_fkm_absolute(obj,q,1);
                x_absoulute_pose = obj.reference_to_first_feethold * first_cortch_to_first_feethold' * obj.frame_bias{2}';
%                 x_absoulute_pose = obj.reference_frame;
                x_relative_pose_1 = raw_fkm_relative(obj,q,1);
                x_relative_pose_2 = raw_fkm_relative(obj,q,2);
                x_relative_pose_3 = raw_fkm_relative(obj,q,3);
                x_relative_pose_4 = raw_fkm_relative(obj,q,4);
                x_relative_pose_5 = raw_fkm_relative(obj,q,5);
                x = [x_absoulute_pose,x_relative_pose_1,x_relative_pose_2,x_relative_pose_3,x_relative_pose_4,x_relative_pose_5];
                %                 error('Need the num of chain');
            end
        end
        
        function x = raw_fkm(obj, q, num_of_chain, ith)
            % Analogous to FKM, but without considering base and end-effector changes.
            % x = RAW_FKM(q, numofchain) calculates the forward kinematics up to the numofchain
            % kinematic chain.
            % x = RAW_FKM(q, numofchain, ith) calculates the forward kinematics up to
            % the ith link of the numofchain kinematic chain.
            % RAW_FKM does not take into account the reference frame.
            
            % By default, the fkm is taken up to the end of the ith
            % kinematic chain.
            partial_chain = false;
            if nargin > 4
                error('Invalid number of arguments');
            elseif nargin == 4
                % the forward kinematics is calculated up to the jth link
                % of the ith kinematic chain.
                if num_of_chain == 0
                    n = 1;
                else
                    n = 2;
                end
                partial_chain = true;
            elseif nargin == 3
                % the forward kinematics is calculated up to the ith
                % kinematic chain
                if num_of_chain == 0
                    n = 1;
                else
                    n = 2;
                end
            else
                error('Need number of chain');
            end
            
            x = DQ(1);
            j =1; % first configuration vector (q1)
            
            % Iterate over the chain
            for i =1:n
                % TODO: The next three lines shouldn't cost much, but this
                % implementation can be improved. For instance, we can
                % store the size of each configuration vector whenever we
                % add a new robot into the serial kinematic chain.
                if i == 1
                    if isa(obj.chain{i}, 'DQ_Kinematics')
                        dim = obj.chain{i}.get_dim_configuration_space();
                        % qi = q(1 : dim);
                        qi = q(j : j + dim - 1);
                        j = j + dim;
                    end
                    if isa(obj.chain{i}, 'DQ')
                        % Is it a rigid transformation? (Rigid transformations are
                        % never reversed in the chain because a reverse rigid
                        % transformation is accomplished by using its
                        % conjugate when adding it to the chain.)
                        x = x*obj.chain{i};
                    else
                        % It's neither a rigid transformation nor a reversed
                        % chain; that is, it's just a regular one.
                        if partial_chain == true
                            x = x*obj.chain{i}.fkm(qi,ith);
                        else
                            x = x*obj.chain{i}.fkm(qi);
                        end
                    end
                elseif i == 2
                    if isa(obj.chain{num_of_chain+1}, 'DQ_Kinematics')
                        dim = obj.chain{num_of_chain+1}.get_dim_configuration_space();
                        % qi = q(1 : dim);
                        qi = q(j : j + dim - 1);
                        j = j + dim;
                    end
                    if isa(obj.chain{num_of_chain+1}, 'DQ')
                        % Is it a rigid transformation? (Rigid transformations are
                        % never reversed in the chain because a reverse rigid
                        % transformation is accomplished by using its
                        % conjugate when adding it to the chain.)
                        x = x*obj.chain{num_of_chain+1};
                    else
                        % It's neither a rigid transformation nor a reversed
                        % chain; that is, it's just a regular one.
                        if partial_chain == true
                            x = x*obj.chain{num_of_chain+1}.fkm(qi,ith);
                        else
                            x = x*obj.chain{num_of_chain+1}.fkm(qi);
                        end
                    end
                end
            end
        end
        
        function set_reference_to_first_feethold(obj, q)
            %             Q = DQ(1);
            Q2 = q(9:11);
%             reference = DQ(1);
            first_cortch_to_first_foothold = raw_fkm_absolute(obj,q,1);
%             x_base_to_first_leg = obj.frame_bias{2}*obj.chain{2}.fkm(Q2);
            x_base_to_first_leg = obj.frame_bias{2}*first_cortch_to_first_foothold;
            reference_to_first_leg = obj.base_frame * x_base_to_first_leg;

            if is_unit(reference_to_first_leg)
                obj.reference_to_first_feethold = reference_to_first_leg;
            else
                error('The base frame must be a unit dual quaternion.');
            end
            %                         vector_from_ref_to_first_feethold = q(1:8);
            %                         reference_to_base = DQ(vector_from_ref_to_first_feethold);
            %                         obj.reference_to_first_feethold = reference_to_base;
        end
        
        function set_reference_to_first_feethold_vrep(obj, foothold1)
             obj.reference_to_first_feethold = foothold1;
        end
        
        function update_base_frame(obj,q)
            Q2 = q(9:11);
%             reference = DQ(1);
            first_cortch_to_first_foothold = raw_fkm_absolute(obj,q,1);
%             x_base_to_first_leg = obj.frame_bias{2}*obj.chain{2}.fkm(Q2);
            x_base_to_first_leg = obj.frame_bias{2}*first_cortch_to_first_foothold;
            obj.base_frame = obj.reference_to_first_feethold * x_base_to_first_leg';
        end
        
        function Trans_feethold1 = get_reference_to_first_feethold(obj)
            Trans_feethold1 = obj.reference_to_first_feethold;
        end
        
        
        function x = raw_fkm_absolute(obj, q, num)
            j = 9 + (num-1)*3;
            dim = obj.chain{num+1}.get_dim_configuration_space();
            Q = q(j : j + dim - 1);
            %             Q = q(9:11);
            x = obj.chain{2}.fkm(Q);
        end
        
        function x = raw_fkm_base(obj,q)
            Q = q(1:8);
            x = DQ(Q);
        end
        
        function x = raw_fkm_relative(obj, q, num)
            if (num > 5)
                error('out of the number of relative dual postion');
            end
            x = DQ(1);
            j = 9 + (num-1)*3; % first configuration vector (q1)
            %             Q = zeros(1,2);
            if isa(obj.chain{num+1}, 'DQ_Kinematics')
                dim = obj.chain{num+1}.get_dim_configuration_space();
                Q1 = q(j : j + dim - 1);
                j = j + dim;
                dim = obj.chain{num+1}.get_dim_configuration_space();
                Q2 = q(j : j + dim - 1);
            end
            
            x_base_to_first_leg = obj.frame_bias{num+1}*obj.chain{num+1}.fkm(Q1);
            x_base_to_second_leg = obj.frame_bias{num+2}*obj.chain{num+2}.fkm(Q2);
            x_second_leg_to_first_leg = x_base_to_second_leg'* x_base_to_first_leg;
            x = x_second_leg_to_first_leg;
        end
        
        function ret = get_dim_configuration_space(obj,num_of_chain)
            % Returns the dimension of the whole-body configuration space.
            if nargin > 2
                error('Invalid number of parameters')
            elseif nargin == 2
                ret = obj.dim_configuration_space{num_of_chain+1};
            elseif nargin == 1
                num = 0;
                ret = 0;
                for i = 1:7
                    num = obj.dim_configuration_space{i};
                    ret = num +ret;
                end
            end
        end
        
        function J = pose_jacobian(obj,q,num_of_chain,ith)
            % Returns the whole-body pose Jacobian.
            %
            % J = POSE_JACOBIAN(q) receives the configuration vector q of the whole
            % kinematic chain and returns the jacobian matrix J that satisfies
            % vec8(xdot) = J * q_dot, where q_dot is the configuration velocity
            % and xdot is the time derivative of the unit dual quaternion that
            % represents the end-effector pose.
            % J = POSE_JACOBIAN(q, num_of_chain) calculates the Jacobian up to the num_of_chain
            % kinematic chain.
            % J = POSE_JACOBIAN(q, num_of_chain,ith) calculates the Jacobian up to the
            % ith link of the num_of_chain kinematic chain.
            
            
            partial_chain = false;
            if nargin > 4
                error('Invalid number of parameters')
            elseif nargin == 4
                % find the Jacobian up to the jth link of the ith
                % intermediate kinematic chain
                partial_chain = true;
                n = num_of_chain;
                J = raw_pose_jacobian(obj,partial_chain,q,n,ith);
            elseif nargin == 3
                % find the Jacobian up to the ith intermediate kinematic
                % chain
                n = num_of_chain;
                J = raw_pose_jacobian(obj,partial_chain,q,n,ith);
            else
                J_absolute = raw_pose_jacobian_absolute(obj,q);
                J_relative_1 = raw_pose_jacobian_relative(obj,q,1);
                J_relative_2 = raw_pose_jacobian_relative(obj,q,2);
                J_relative_3 = raw_pose_jacobian_relative(obj,q,3);
                J_relative_4 = raw_pose_jacobian_relative(obj,q,4);
                J_relative_5 = raw_pose_jacobian_relative(obj,q,5);
                J = zeros(48,26);
                for i = 1:size(J_absolute,1)
                    for j = 1:size(J_absolute,2)
                        J(i,j) = J_absolute(i,j);
                    end
                end
                
                for i = 1:8
                    for j = 1:6
                        J(8*1+i,5+3*1+j) = J_relative_1(i,j);
                        J(8*2+i,5+3*2+j) = J_relative_2(i,j);
                        J(8*3+i,5+3*3+j) = J_relative_3(i,j);
                        J(8*4+i,5+3*4+j) = J_relative_4(i,j);
                        J(8*5+i,5+3*5+j) = J_relative_5(i,j);
                    end
                end
                
                %                 J = [J_absolute,J_relative_1,J_relative_2,J_relative_3,J_relative_4,J_relative_5];
            end
        end
        
        
        function J = raw_pose_jacobian(obj,partial_chain,q,n,ith)
            if partial_chain == true
                x_0_to_n = obj.fkm(q,n,ith);
            else
                x_0_to_n = obj.fkm(q,n);
            end
            
            j = 1;
            
            for i = 0:n-1
                if partial_chain == true && i == n-1
                    x_0_to_iplus1 = obj.fkm(q,i+1,ith);
                else
                    x_0_to_iplus1 = obj.fkm(q,i+1);
                end
                x_iplus1_to_n = x_0_to_iplus1'*x_0_to_n;
                
                % Constant rigid transformations in the chain do not change the
                % dimension of the configuration space.
                if isa(obj.chain{i+1}, 'DQ_Kinematics')
                    
                    
                    dim = obj.chain{n}.get_dim_configuration_space();
                    q_iplus1 = q(1 : dim);
                    
                    % TODO: The code below can be cleaned up to prevent
                    % duplication.
                    
                    
                    if partial_chain == true
                        L{i+1} = hamiplus8(obj.fkm(q,i)) * ...
                            haminus8(x_iplus1_to_n) * ...
                            obj.chain{i+1}.pose_jacobian(q_iplus1,ith);
                    else
                        L{i+1} = hamiplus8(obj.fkm(q,i)) * ...
                            haminus8(x_iplus1_to_n) * ...
                            obj.chain{i+1}.pose_jacobian(q_iplus1);
                    end
                end
            end
            J = cell2mat(L);
        end
        
        function J = raw_pose_jacobian_relative(obj,q,num)
            if (num > 5)
                error('out of the number of relative dual postion');
            end
            x = DQ(1);
            j =1; % first configuration vector (q1)
            %             Q = zeros(1,2);
            if isa(obj.chain{num+1}, 'DQ_Kinematics')
                dim = obj.chain{num+1}.get_dim_configuration_space();
                Q1 = q(j : j + dim - 1);
                j = j + dim;
                dim = obj.chain{num+1}.get_dim_configuration_space();
                Q2 = q(j : j + dim - 1);
            else
                error('wrong type')
            end
            x_second_crotch_to_second_leg = obj.chain{num+2}.fkm(Q2);
            x_first_crotch_to_first_leg = obj.chain{num+1}.fkm(Q1);
            x_1 = x_second_crotch_to_second_leg'* obj.frame_bias{num+2}' * obj.frame_bias{num+1};
            J1 = hamiplus8(x_1) * obj.chain{num+1}.pose_jacobian(Q1);
            x_2 = obj.frame_bias{num+2}' * obj.frame_bias{num+1} * x_first_crotch_to_first_leg;
            a = obj.chain{num+1}.pose_jacobian(Q2);
            J2= haminus8(x_2)*(dualconj(obj,num,Q2));
            J = [J1,J2];
            %             J = cell2mat(L);
            
        end
        
        function J = dualconj(obj,num,q)
            matrix = obj.chain{num+1}.pose_jacobian(q);
            J = matrix;
            if (size(matrix,1) ~= 8)
                error('error dual matrix size')
            else
                list = size(matrix,2);
                for i = 1:list
                    J(2,i) = -matrix(2,i);
                    J(3,i) = -matrix(3,i);
                    J(4,i) = -matrix(4,i);
                    J(6,i) = -matrix(6,i);
                    J(7,i) = -matrix(7,i);
                    J(8,i) = -matrix(8,i);
                end
            end
        end
        
        function J = raw_pose_jacobian_absolute(obj,q)
            q1 = q(9:11);
            first_cortch_to_first_feethold = raw_fkm_absolute(obj,q,1);
            x_first_feethold_to_firstbase = first_cortch_to_first_feethold' * obj.frame_bias{2}';
            x_ref_to_feethold1 = obj.reference_to_first_feethold;
            J1 = haminus8(x_first_feethold_to_firstbase);
            J2 = hamiplus8(x_ref_to_feethold1) * haminus8(conj(obj.frame_bias{2})) * dualconj(obj,1,q1);
                
%             x_first_crotch_to_first_leg = obj.chain{2}.fkm(q1);
%             x_first_leg_to_first_crotch = conj(x_first_crotch_to_first_leg);
%             x_1 = x_first_leg_to_first_crotch * conj(obj.frame_bias{2});
%             J1 = haminus8(x_1);
%             x_ref_to_base = obj.reference_frame * obj.frame_bias{1} * raw_fkm(obj,q,0);
%             J2 = hamiplus8(x_ref_to_base) * haminus8(conj(obj.frame_bias{2})) * dualconj(obj,1,q1);
            J = [J1, J2];
        end
        
        %         function [Q1, Q2] = get_dim_configuration_space_for_two_legs(obj,q,num)
        %             j =1;
        %             if isa(obj.chain{1}, 'DQ_Kinematics')
        %                 dim = obj.chain{1}.get_dim_configuration_space();
        %                 Q1 = q(j : j + dim - 1);
        %                 j = j + dim;
        %                 dim = obj.chain{num+1}.get_dim_configuration_space();
        %                 Q2 = q(j : j + dim - 1);
        %             else
        %                 error('wrong type')
        %             end
        %         end
        
        function [constraint_matrix,constraint_vector] = get_constraint_matrix_and_vector(obj,q,base_vec)
            
            first_cortch_to_first_feethold = raw_fkm_absolute(obj,q,1);
            x_ref_to_base = obj.reference_to_first_feethold * first_cortch_to_first_feethold' * obj.frame_bias{2}';
            %             J_trans = zeros(16,32);
            %             J_MN = zeros(32,26);
            
            J_trans = zeros(24,48);
            J_MN = zeros(48,26);
            for i = 1:6
                qi = q(6+3*i:8+3*i);
                x_base_to_crotch_i = obj.frame_bias{i};
                x_crotch_to_feet_i = obj.chain{i+1}.fkm(qi);
                x_ref_to_crotch_i = x_ref_to_base*x_base_to_crotch_i;
                x_ref_to_feet_i = x_ref_to_base * x_base_to_crotch_i * x_crotch_to_feet_i;
                x_base_to_feet_i = x_base_to_crotch_i * x_crotch_to_feet_i;
                
                J_trans_i = get_J_trans_matrix(x_ref_to_feet_i);
                M_i = haminus8(x_base_to_feet_i);
                N_i = hamiplus8(x_ref_to_crotch_i) * obj.chain{i+1}.pose_jacobian(qi);
                
                J_trans(4*i-3:4*i,8*i-7:8*i) = J_trans_i;
                J_MN(8*i-7:8*i,1:8) = M_i;
                J_MN(8*i-7:8*i,3*i+6:3*i+8) = N_i;
                
                
                j1 = haminus8(x_base_to_feet_i);
                %                 vec_ref_to_feet = j1 * vec8(base_vec);
                %                 V(8*i-7:8*i) = -vec_ref_to_feet;
                % V(8*i-7+8:8*i+8) = -vec_ref_to_feet;
            end
            J_whole = 2 * J_trans * J_MN;
            constraint_vector = J_whole(5:24,1:8) * base_vec.q;
            constraint_matrix = J_whole(5:24,9:26);
%             constraint_vector = J_whole(:,1:8) * base_vec.q;  
%             constraint_matrix = J_whole(:,9:26);
            
        end
        
        function V = get_constraint_vector(obj,q,base_vec)
            V= zeros(1,48);
            %             V = zeros(1,56);
            %             for j = 1:8
            %                 V(j) = 0;
            %             end
            for i = 1:6
                qi = q(6+3*i:8+3*i);
                x_base_to_crotch = obj.frame_bias{i};
                x_crotch_to_feet = obj.chain{i+1}.fkm(qi);
                x_base_to_feet = x_base_to_crotch * x_crotch_to_feet;
                j1 = haminus8(x_base_to_feet);
                vec_ref_to_feet = j1 * vec8(base_vec);
                V(8*i-7:8*i) = -vec_ref_to_feet;
                a
                %                 V(8*i-7+8:8*i+8) = -vec_ref_to_feet;
            end
        end
        
        function [x_current_abs_pose,EBV] = get_estimated_base(obj,q,last_q, sampling_time)
            if nargin > 4
                error('Invalid number of arguments');
                %             elseif nargin == 4
                %                 x = obj.reference_frame * obj.frame_bias{num_of_chain + 1} * raw_fkm(obj,q,num_of_chain,ith);
                %             elseif nargin == 3
                %                 if num_of_chain ~= 0
                %                     x = obj.reference_frame * obj.frame_bias{num_of_chain + 1} * raw_fkm_absolute(obj,q,num_of_chain);
                %                 else
                %                     x = obj.reference_frame * obj.frame_bias{num_of_chain + 1} * raw_fkm_base(obj,q);
                %                 end
            else
%                 first_cortch_to_first_feethold_cur = raw_fkm_absolute(obj,q,1);
%                 x_current_abs_pose = obj.reference_frame * obj.reference_to_first_feethold * first_cortch_to_first_feethold_cur' * obj.frame_bias{2}';

                first_cortch_to_first_feethold_cur = raw_fkm_absolute(obj,q,1);
                x_current_abs_pose = obj.reference_to_first_feethold * first_cortch_to_first_feethold_cur' * obj.frame_bias{2}';
%                 debug_c =   obj.reference_to_first_feethold * obj.base_frame' * first_cortch_to_first_feethold_cur' * obj.frame_bias{2}'
                x_last_abs_pose = DQ(last_q(1:8));
           
                %     get the transformation dq from last pose to current pose
                x_trans = x_last_abs_pose'* x_current_abs_pose;
                EBV = (log(x_trans) * x_current_abs_pose) / sampling_time;
            end
        end
        
        function plot(obj,q)
            % Draws the whole kinematic chain.
            %
            % PLOT(q) draws the whole kinematic chain, given 'q'. It does
            % all necessary transformations to take into account reverse
            % chains and direct kinematic chains.
            
            if isa(obj.chain{1}, 'DQ_Kinematics')
                dim_conf_space = obj.chain{1}.get_dim_configuration_space();
                %DQ_MobileBase does not have 'nojoints' property
                if isa(obj.chain{1}, 'DQ_MobileBase')
                    %                     plot(obj.chain{1},q(1:8));
                    
                    % % % % %            plot a plane base           % % % % % % % % % % % % % %
%                     dq_pose = obj.fkm(q,0);
%                     vec_trans = vec4(translation(dq_pose));
%                     pos = vec_trans(2:4);
%                     
%                     current_base_frame1 = obj.fkm(q,0)* obj.frame_bias{2};
%                     current_base_frame2 = obj.fkm(q,0)* obj.frame_bias{4};
%                     current_base_frame3 = obj.fkm(q,0)* obj.frame_bias{6};
%                     vec_trans1 = vec4(translation(current_base_frame1));
%                     p1 = vec_trans1(2:4);
%                     vec_trans2 = vec4(translation(current_base_frame2));
%                     p2 = vec_trans2(2:4);
%                     vec_trans3 = vec4(translation(current_base_frame3));
%                     p3 = vec_trans3(2:4);
%                     plot_line(p1, p2, p3,pos);                    
                    % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  % % % % %
                    
                    % % % % %            plot a circle base        % % % % % % % % % % % % % % %
                    
                    %                     rad = 0.25;
                    %                     dq_pose = obj.fkm(q,0);
                    %                     vec_trans = vec4(translation(dq_pose));
                    %                     trans = vec_trans(2:4);
                    %                     vec_rota_axis = vec4(rotation_axis(dq_pose));
                    %                     n = vec_rota_axis(1:4);
                    %
                    % %                     vec_rotation = vec4(rotation(dq_pose))
                    % %                     n = vec_rotation(1:4);
                    % %                     x = n(2) / sqrt(1-n(1)*(1))
                    % %                     y = n(3) / sqrt(1-n(1)*(1))
                    % %                     z = n(4)/ sqrt(1-n(1)*(1))
                    % %                     m = [x,y,z];
                    %
                    %                     drawCircle(rad,trans,m,'r')
                    
                    % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
                elseif isa(obj.chain{1}, 'DQ_SerialManipulator')
                    % To improve plot performance, specially for very large
                    % configuration space dimensions, we never plot the
                    % joints of serial manipulators.
                    
                    % If the first kinematic chain is a fixed-base serial
                    % chain *and* reversed, we must adapt its base frame so
                    % that the DQ_Kinematics/plot function, which always
                    % start ploting from its base frame, plots the serial
                    % chain with the end-effector coinciding with the
                    % whole-body base frame. (Note that each individual chain
                    % has its own base frame used to determine its spatial
                    % location).
                    if obj.reversed(1)
                        current_base_frame = obj.get_base_frame() * ...
                            obj.raw_fkm(q,1);
                    else
                        % if the chain is not reversed, then its base frame
                        % must coincide with the whole-body base frame.
                        current_base_frame = obj.get_base_frame();
                    end
                    obj.chain{1}.set_base_frame(current_base_frame);
                    plot(obj.chain{1},q(1:dim_conf_space),'nojoints');
                    hold on;
                end
                j = 8 + 1;
            else
                j = 1;
            end
            
            % Iterate over the chain
            for i = 2:length(obj.chain)
                % If the first element in the kinematic chain is a mobile
                % base, its fkm coincides with the base location, already
                % considering a frame displacement, if aplicable (e.g., in case
                % the mobile base frame is not in its center).
                
                %                 if isa(obj.chain{1}, 'DQ_MobileBase')
                %                     current_base_frame = obj.fkm(q,0);
                %                 else
                % The first element in the chain has a fixed-base robot,
                % which may be located arbitrarily in the workspace with a
                % rigid transformation given by base_frame (which is not
                % necessarily the same as the reference frame).
                
                current_base_frame = DQ(1)* obj.frame_bias{i};
%                 current_base_frame = obj.fkm(q,0)* obj.frame_bias{i};
                %                 end
                
                % Constant rigid transformations do not change the
                % dimension of the configuration space. Furthermore, we do
                % not plot them (this behavior may be changed in the future,
                % though).
                if isa(obj.chain{i}, 'DQ_Kinematics')
                    obj.chain{i}.set_base_frame(current_base_frame);
                    
                    dim = obj.chain{i}.get_dim_configuration_space();
                    qi = q(j : j + dim - 1);
                    j = j + dim;
                    
                    % We do not plot names, bases, and coordinate systems of
                    % the intermediate kinematic chains.
                    if i < length(obj.chain)
                        hold on;
                        plot(obj.chain{i},qi, 'nobase', ...
                            'nowrist', 'noname', 'nojoints');
                    else
                        % But we plot the coordinate system of the whole-body
                        % end-effector.
                        plot(obj.chain{i},qi, 'nobase', 'noname', 'nojoints');
                        hold on;
                    end
                end
            end
            
        end

        %         function N_i = get_N_matrix(obj,dq1,qi,num)
        %             N_i = zeros(8,3);
        %             B = zeros(4,8);
        %             C = zeros(4,8);
        %             B(:,5:8) = eye(4);
        %             C(:,1:4) = dq1.C4;
        %             J_qi = obj.chain{num+1}.pose_jacobian(qi);
        %
        %             N_i(1:4,:) = B*hamiplus8(dq1)*J_qi;
        %             N_i(5:8,:) = C*hamiplus8(dq1)*J_qi;
        %         end
        
        
        
    end
end

function J_trans_i = get_J_trans_matrix(x_ref_to_feet_i)
J_trans_i = zeros(4,8);
x_ref_to_feet_i_pure_part = x_ref_to_feet_i.P;
J_trans_i(:,1:4)= hamiplus4(x_ref_to_feet_i.D) * x_ref_to_feet_i.C4;
J_trans_i(:,5:8)= haminus4(x_ref_to_feet_i_pure_part');
end

function drawCircle(rad,pos,n,color)
%https://demonstrations.wolfram.com/ParametricEquationOfACircleIn3D/
%draws a 3D circle at position pos with radius rad, normal to the
%circle n, and color color.
phi = atan2(n(2),n(1)); %azimuth angle, in [-pi, pi]
theta = atan2(sqrt(n(1)^2 + n(2)^2) ,n(3));% zenith angle, in [0,pi]
t = 0:pi/32:2*pi;
x = pos(1)- rad*( cos(t)*sin(phi) + sin(t)*cos(theta)*cos(phi) );
y = pos(2)+ rad*( cos(t)*cos(phi) - sin(t)*cos(theta)*sin(phi) );
z = pos(3)+ rad*sin(t)*sin(theta);
plot3(x,y,z,color)
end

function [normal, d] = plot_line(p1, p2, p3,pos)
% This function plots a line from three points.
% I/P arguments:
%   p1, p2, p3 eg, p1 = [x y z]
%
%
% O/P is:
% normal: it contains a,b,c coeff , normal = [a b c]
% d : coeff
normal = cross(p1 - p2, p1 - p3);
d = p1(1)*normal(1) + p1(2)*normal(2) + p1(3)*normal(3);
d = -d;
x = pos(1)-0.2:0.04:pos(1)+0.2; y = pos(2)-0.2:0.04:pos(1)+0.2;
[X,Y] = meshgrid(x,y);
Z = (-d - (normal(1)*X) - (normal(2)*Y))/normal(3);
mesh(X,Y,Z)
end

% function M_i = get_M_matirx(dq1)
% M_i = zeros(8,8);
% B = zeros(4,8);
% C = zeros(4,8);
% B(:,5:8) = eye(4);
% C(:,1:4) = dq1.C4;
% % dq1_a = haminus8(dq1);
% M_i(1:4,:) = B*haminus8(dq1);
% M_i(5:8,:) = C*haminus8(dq1);
%
% end

