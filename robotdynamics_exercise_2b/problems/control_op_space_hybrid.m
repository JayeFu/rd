function [ tau ] = control_op_space_hybrid( I_r_IE_des, eul_IE_des, q, dq, I_F_E_x )
% CONTROL_OP_SPACE_HYBRID Operational-space inverse dynamics controller 
% with a PD stabilizing feedback term and a desired end-effector force.
%
% I_r_IE_des --> a vector in R^3 which describes the desired position of the
%   end-effector w.r.t. the inertial frame expressed in the inertial frame.
% eul_IE_des --> a set of Euler Angles XYZ which describe the desired
%   end-effector orientation w.r.t. the inertial frame.
% q --> a vector in R^n of measured joint positions
% q_dot --> a vector in R^n of measured joint velocities
% I_F_E_x --> a scalar value which describes a desired force in the x
%   direction

% Design the control gains
kp = 50.0;
kd = 14.0;
kpMat = kp * diag([1.0 1.0 1.0 1.0 1.0 1.0]);
kdMat = kd * diag([1.0 1.0 1.0 1.0 1.0 1.0]);

% Desired end-effector force
I_F_E = [I_F_E_x, 0.0, 0.0, 0.0, 0.0, 0.0]';

% Find jacobians, positions and orientation
I_J_e = I_Je_fun_solution(q);
I_dJ_e = I_dJe_fun_solution(q, dq);
T_IE = T_IE_fun_solution(q);
I_r_IE = T_IE(1:3, 4);
C_IE = T_IE(1:3, 1:3);

% Define error orientation using the rotational vector parameterization.
C_IE_des = eulAngXyzToRotMat(eul_IE_des);
C_err = C_IE_des*C_IE';
orientation_error = rotMatToRotVec_solution(C_err);

% Define the pose error.
chi_err = [I_r_IE_des - I_r_IE;
           orientation_error];

% Project the joint-space dynamics to the operational space
% TODO

% calculting M, b, g for further use
M = M_fun_solution(q);
b = b_fun_solution(q, dq);
g = g_fun_solution(q);

% according to (3.87)-(3.89)
M_inv = inv(M);
lambda = inv(I_J_e * M_inv * I_J_e');;
mu = lambda * I_J_e * M_inv * b - lambda * I_dJ_e * dq;
p =  lambda * I_J_e * M_inv * g;

% according to (3.91)
w = I_J_e * dq;
w_dot = kpMat * chi_err + kdMat * (-w);

% Define the motion and force selection matrices.
% TODO
Sigma_p = [0, 0, 0;
    0, 1, 0;
    0, 0, 1];
Sigma_r = [1, 0, 0;
    0, 1, 0;
    0, 0, 1];
Sm = [Sigma_p, zeros(3,3);
    zeros(3,3), Sigma_r];
Sf = [eye(3) - Sigma_p, zeros(3,3);
    zeros(3,3), eye(3) - Sigma_r];
% Design a controller which implements the operational-space inverse
% dynamics and exerts a desired force.
tau = zeros(6,1); % TODO
tau = I_J_e' * (lambda * Sm * w_dot + Sf * I_F_E + mu + p);
end
