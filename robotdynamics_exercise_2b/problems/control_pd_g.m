function [ tau ] = control_pd_g( q_des, q, q_dot )
% CONTROL_PD_G Joint space PD controller with gravity compensation.
%
% q_des --> a vector R^n of desired joint angles.
% q --> a vector R^n of measured joint angles.
% q_dot --> a vector in R^n of measured joint velocities

% Gains 
% Here the controller response is mainly inertia dependent
% so the gains have to be tuned joint-wise
kp = 10.0;
kd = 2.0;
kpMat = kp * diag([5000 3000 5 1 0.5 0.01]);
kdMat = kd * diag([5000 3000 5 1 0.5 0.01]);

% The control action has a gravity compensation term, as well as a PD
% feedback action which depends on the current state and the desired
% configuration.
tau = zeros(6,1); % TODO

% maybe we need to manually define the desiredjoint velocities
joint_des_vel = 0.0;
q_dot_des = repelem(joint_des_vel, 6)';

% According to (3.75)
g = g_fun_solution(q);
tau = kpMat * (q_des - q) + kdMat * (q_dot_des - q_dot) + g;


end