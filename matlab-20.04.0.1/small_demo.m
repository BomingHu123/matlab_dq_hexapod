% current_absolute_pose_dq = x_current(1);
% current_absolute_pose = vec8(current_absolute_pose_dq);

include_namespace_dq
r =cos(1) + i_*sin(1)+ j_*sin(1) + k_*sin(1);
p = 1*i_ + 0*j_ + 1*k_;
a = r + E_*0.5*p*r;

% b = translation(a)

% rot_axis = rotation_axis(a);
% rot_angle = rotation_angle(a);
% trans_part = translation(a);
% vec_r = vec3(rot_axis)
% vec_t = vec3(trans_part)
% a = cross(vec_t,vec_r);
% c = Q8(a)
% b = DQ(vec_t)
% T = 1000;
% b = [1,0,0,0,0,0,0,0];
% dq_b = DQ(b);
% 
% res = 1/T * log(a)*dq_b
% 
% is_unit(res)