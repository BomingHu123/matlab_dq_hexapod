% vi = DQ_VrepInterface;
% vi.connect('127.0.0.1',19997);
% vi.start_simulation();
sampling_time = 0.05;
total_time = 200;
for t=0:sampling_time:total_time
%% Get obstacles from V-REP
plane = get_plane_from_vrep(vi,'ObstaclePlane',DQ.k);
cylinder1 = get_line_from_vrep(vi,'ObstacleCylinder1',DQ.k);
cylinder2 = get_line_from_vrep(vi,'ObstacleCylinder2',DQ.k);
%% Set references for both robots
[lwr4_xd, lwr4_ff] = compute_lwr4_reference(lwr4 ,...
    simulation_parameters , lwr4_x0, t);
[youbot_xd , youbot_ff] = compute_youbot_reference(...
    youbot_control , lwr4_xd, lwr4_ff);
%% Compute the control input for the manipulator
lwr4_u = lwr4_controller.compute_tracking_control_signal(...
    lwr4_q, vec8(lwr4_xd),vec8(lwr4_ff));
%% Compute constrained control input for the youbot
[Jconstraint , bconstraint] = compute_constraints(youbot, ...
    youbot_q , plane,cylinder1 ,cylinder2);
youbot_control.set_inequality_constraint(-Jconstraint ,...
    1*bconstraint);
youbot_u= youbot_control.compute_tracking_control_signal(...
    youbot_q , vec8(youbot_xd), vec8(youbot_ff));
% Since we are using V-REP just for visualization , integrate
% the control signal to update the robots configurations
lwr4_q = lwr4_q + sampling_time*lwr4_u;
youbot_q = youbot_q + sampling_time*youbot_u;
%% Send desired values to V-REP
lwr4_vreprobot.send_q_to_vrep(lwr4_q);
youbot_vreprobot.send_q_to_vrep(youbot_q);
end