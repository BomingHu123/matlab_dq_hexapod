clc;
clear;
include_namespace_dq

q = [0, 0.3770, 0.1257, -0.5655, 0, 0, 0]';
t = 0.001;% sampling time
T = 0;
Q = [];
Time =[];
gain = 10;% controller gain
r =cos(pi/2) + j_*sin(pi/2);
p = 0.1*i_ + 0.2*j_ + 0.3*k_;
xd = r + E_*0.5*p*r;

solver = DQ_QuadprogSolver ;

lwr4 = KukaLwr4Robot.kinematics();
lwr4.plot(q);
control = DQ_PseudoinverseController(lwr4);
% control = DQ_ClassicQPController(lwr4,solver);
control.set_control_objective(ControlObjective.Pose);
control.set_gain(gain);
control.set_stability_threshold(0.0001);
while ~control.system_reached_stable_region()
    u = control.compute_setpoint_control_signal(q,vec8(xd));
    q = q + t*u;
    Q=[Q,q];
    T = T + t;
    Time = [Time T];
end
% hold on;
% plot(Time,Q(1,:))
% plot(Time,Q(2,:))
% plot(Time,Q(3,:))
% plot(Time,Q(4,:))
% plot(Time,Q(5,:))
% plot(Time,Q(6,:))
% plot(Time,Q(7,:))