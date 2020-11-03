function [ tau ] = control_inv_dyn(I_r_IE_des, eul_IE_des, q, q_dot)
% CONTROL_INV_DYN Operational-space inverse dynamics controller with a PD 
% stabilizing feedback term.
%
% I_r_IE_des --> a vector in R^3 which describes the desired position of the
%   end-effector w.r.t. the inertial frame expressed in the inertial frame.
% eul_IE_des --> a set of Euler Angles XYZ which describe the desired
%   end-effector orientation w.r.t. the inertial frame.
% q --> a vector in R^n of measured joint angles
% q_dot --> a vector in R^n of measured joint velocities

% Set the joint-space control gains.
kp = 10.0;
kd = 6.0;
kpMat = kp * diag([1.0 1.0 1.0 1.0 1.0 1.0]);
kdMat = kd * diag([1.0 1.0 1.0 1.0 1.0 1.0]);

% Find jacobians, positions and orientation based on the current
% measurements.
I_J_e = I_Je_fun_solution(q);
I_dJ_e = I_dJe_fun_solution(q, q_dot);
T_IE = T_IE_fun_solution(q);
I_r_Ie = T_IE(1:3, 4);
C_IE = T_IE(1:3, 1:3);

% Define error orientation using the rotational vector parameterization.
C_IE_des = eulAngXyzToRotMat(eul_IE_des);
C_err = C_IE_des*C_IE';
orientation_error = rotMatToRotVec_solution(C_err);

% Define the pose error.
chi_err = [I_r_IE_des - I_r_Ie;
           orientation_error];

% PD law, the orientation feedback is a torque around error rotation axis
% proportional to the error angle.
tau = zeros(6, 1); % TODO

% calculting M, b, g for further use
M = M_fun_solution(q);
b = b_fun_solution(q, q_dot);
g = g_fun_solution(q);

% according to (3.87)-(3.89)
M_inv = inv(M);
Lambda = inv(I_J_e * M_inv * I_J_e');
mu = Lambda * I_J_e * M_inv * b - Lambda * I_dJ_e * q_dot;
p = Lambda * I_J_e * M_inv * g;

% according to (3.91)
w = I_J_e * q_dot;
w_dot = kpMat * chi_err + kdMat * (-w);

% according to (3.90)
tau = I_J_e' * (Lambda * w_dot + mu + p);

end