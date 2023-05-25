clc; clear all;
include_namespace_dq

sampling_time = 0.005;
r0 = cos(0) + (cos(0)/sqrt(2) * i_ + sin(0) * j_ + cos(0)/sqrt(2) * k_)*sin(0);
p0 = cos(0)*i_ + sin(0)*j_ + 0*k_;
x0 = r0 + E_*0.5*p0*r0;
current_x = x0;
last_x = x0;
total_time =5;
ii = 1;
max_iterations = round(total_time/sampling_time);
Position_matrix = zeros(3,max_iterations);
X_direction_matrix = zeros(3,max_iterations);
Y_direction_matrix = zeros(3,max_iterations);
Z_direction_matrix = zeros(3,max_iterations);
Kxi_matrix = zeros(6,max_iterations);
Kxi_calcul_matrix = zeros(6,max_iterations);

for t =0:sampling_time:total_time
rt =cos(t) + (cos(t)/sqrt(2) * i_ + sin(t) * j_ + cos(t)/sqrt(2)* k_)*sin(t);
pt = cos(t)*i_ + sin(t)*j_ + t*k_;
xt = rt + E_*0.5*pt*rt;


nt = cos(t)/sqrt(2) * i_ + sin(t) * j_ + cos(t)/sqrt(2) * k_;
nt_dot = -sin(t)/sqrt(2) *i_ + cos(t) *j_ - sin(t)/sqrt(2) * k_;
w_t = 2 * (nt+nt_dot*sin(t)*cos(t)-nt_dot*nt*(sin(t))^2);
v_t = -sin(t) * i_ + cos(t) * j_ + k_;
kxi_t = w_t + E_ * (v_t+cross(pt,w_t));

% current_x = normalize(xt)
current_x = xt;
current_x_approximation = exp(sampling_time*kxi_t/2) * last_x;

[x_dot,kxi_calculate_1] = compute_estimated_velocity(current_x,last_x,sampling_time);


% x_trans =  last_x' * current_x;
% kxi = 2*log(x_trans)/ sampling_time;
% kxi_calculate_1 = Ad(current_x,kxi);

% x_trans = Ad(current_x,x_trans);
% kxi_calculate_1 = 2*log(x_trans)/ sampling_time;

% a = exp(sampling_time*kxi_t/2);


last_x = current_x;

Position_matrix(:,ii) = vec3(pt);
rotation_quaternion = vec4(rt);
Rotation_matrix = quat2rotm(rotation_quaternion');
X_direction_matrix(:,ii) = Rotation_matrix(:,1)+Position_matrix(:,ii);
Y_direction_matrix(:,ii) = Rotation_matrix(:,2)+Position_matrix(:,ii);
Z_direction_matrix(:,ii) = Rotation_matrix(:,3)+Position_matrix(:,ii);

Kxi_matrix(:,ii) = vec6(kxi_t);
Kxi_calcul_matrix(:,ii) = vec6(kxi_calculate_1);

ii = ii + 1;



end


for i = 1:10:length(Position_matrix(1,:))
hold on; 
plot3([Position_matrix(1,i),X_direction_matrix(1,i)],[Position_matrix(2,i),X_direction_matrix(2,i)],[Position_matrix(3,i),X_direction_matrix(3,i)],'r');
hold on; 
plot3([Position_matrix(1,i),Y_direction_matrix(1,i)],[Position_matrix(2,i),Y_direction_matrix(2,i)],[Position_matrix(3,i),Y_direction_matrix(3,i)],'g');
hold on; 
plot3([Position_matrix(1,i),Z_direction_matrix(1,i)],[Position_matrix(2,i),Z_direction_matrix(2,i)],[Position_matrix(3,i),Z_direction_matrix(3,i)],'b');
hold on; 
end

% T =1:sampling_time:total_time;
% for i = 1:6
% figure(i+1)
% plot(T(2:length(T)),Kxi_matrix(i,2:length(T)));
% xlabel('Time/s') 
% hold on;
% plot(T(2:length(T)),Kxi_calcul_matrix(i,2:length(T)));
% xlabel('Time/s') 
% end



function [estimated_velocity, kxi] = compute_estimated_velocity(current_pose,last_pose,sampling_time)
x_trans =  last_pose' * current_pose;

kxi_1 = 2*log(x_trans)/ sampling_time;
% kxi_2 = 2 * log(last_pose) /sampling_time
% kxi = kxi_2 + Ad(last_pose,kxi_1);
kxi = Ad(last_pose,kxi_1);
estimated_velocity = 0.5 * kxi * current_pose;
end