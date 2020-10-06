function [ Dq ] = kinematicMotionControl(q, r_des, v_des)
% Inputs:
%  q          : current configuration of the robot
% r_des       : desired end effector position
% v_des       : desired end effector velocity
% Output: joint-space velocity command of the robot.

% Compute the updated joint velocities. This would be used for a velocity controllable robot
% TODO:
Dq = 0.1*ones(6,1);

% Going to use PID control to get desired command velocity
% get current position r_cur
r_cur = jointToPosition(q);

% Kp for PID control
Kp = 5;

% compute command velocity
v_command = v_des+Kp*(r_des-r_cur);
% expand linear velocity to rotation velocity
w_command = [v_command;zeros(3,1)];

% get current Jacobian matrix
J_cur = jointToJac(q);

% Damping ratio
lambda = 0.01;

% map velocities to joint velocties
Dq = pseudoInverseMat(J_cur,lambda)*w_command;
end
