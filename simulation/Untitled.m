% arm_DH_theta = [0,0,0];
%             arm_DH_d = [-0.0112, 0, 0];
%             arm_DH_a = [0.0503, 0.0724, 0];
%             arm_DH_alpha = [-pi/2, 0,0];

arm_DH_theta = [0,0,0];
            arm_DH_d = [-0.0112, 0, 0];
            arm_DH_a = [0.0503, 0.0724, 0];
            arm_DH_alpha = [-pi/2, 0,0];
            
            arm_DH_matrix = [arm_DH_theta; arm_DH_d; arm_DH_a;arm_DH_alpha];
            leg = DQ_SerialManipulator(arm_DH_matrix ,'standard');

include_namespace_dq
r =cos(pi/4) + i_*sin(pi/4);
p = 0.116*i_ + -0.01105*j_ + 0*k_;
dq_x = r + E_*0.5*p*r;

q = [0,0,0]/57.3

a = leg.fkm(q);
b = a * dq_x;
a_quat = vec4(a.rotation);
[x_a,y_a,z_a] = quat2angle(a_quat','xyz')
b_quat = vec4(b.rotation);
[x_b,y_b,z_b] = quat2angle(b_quat','xyz')
