% current_absolute_pose_dq = x_current(1);
% current_absolute_pose = vec8(current_absolute_pose_dq);

include_namespace_dq
r =cos(1) + k_*sin(1);
p = 0*i_ + 0*j_ + 0*k_;
a = r + E_*0.5*p*r;

a = [1,1,0,0,0,0,0,1];
dq_a =a(1) + a(2)*i_ + a(3)*j_ + a(4)*k_ + E_ * (a(5) + a(6)*i_ + a(7)*j_ + a(8)*k_);
vec8(dq_a)

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