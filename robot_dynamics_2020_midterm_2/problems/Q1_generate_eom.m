% generate equations of motion
function eom = Q1_generate_eom(gc, kin, params, jac)
% By calling:
%   eom = generate_eom(gc, kin, params, jac)
% a struct 'eom' is returned that contains the matrices and vectors
% necessary to compute the equations of motion.
%
% Inputs:
%   - gc        : Current generalized coordinates (q, dq)
%   - kin       : Struct containing symbolic expresses for the kinematics
%   - params    : Struct with parameters
%   - jac       : Struct containing symbolic expresses for the jacobians
%   
% Output:
%   - eom       : Struct with fields {M, b, g}, implementing the system dynamics
%

%% Setup
q = gc.q;      % Generalized coordinates (3x1 sym)
dq = gc.dq;    % Generalized velocities (3x1 sym)

T_Ik = kin.T_Ik;        % Homogeneous transforms (3x1 cell)->(4x4 sym)
R_Ik = kin.R_Ik;        % Rotation matrices (3x1 cell)->(3x3 sym)

k_I_s = params.k_I_s;      % Inertia tensor of body k in frame k (3x1 cell)->(3x3 sym)
m = params.m;              % Mass of body k (3x1 cell)->(1x1 double)
I_g_acc = params.I_g_acc;  % Gravitational acceleration in inertial frame (3x1 double)
k_r_ks = params.k_r_ks;    % CoM location of body k in frame k (3x1 cell)->(3x1 double)

I_Jp = jac.I_Jp;    % CoM Positional Jacobian in frame I (3x1 cell)->(3x6 sym)
I_Jr = jac.I_Jr;        % CoM Rotational Jacobian in frame I (3x1 cell)->(3x6 sym)

eom.M = sym(zeros(3,3));
eom.g = sym(zeros(3,1));
eom.b = sym(zeros(3,1));

%% Compute mass matrix
fprintf('Computing mass matrix M... ');
M = sym(zeros(3,3));
%TODO: Implement M = ...;

for i=1:3
    fprintf('M%i...', i);
    % Jacobians
    I_Js_i = I_Jp{i};
    I_Jr_i = I_Jr{i};
    % mass
    m_i = m{i};
    % intertia tensor
    i_I_s_i = k_I_s{i};
    % rot mat
    R_Ii = R_Ik{i};
    % inertia tensor in frame I
    I_I_s_i = R_Ii * i_I_s_i * R_Ii';
    % cal M
    M = M + I_Js_i' * m_i * I_Js_i + I_Jr_i'* I_I_s_i * I_Jr_i;
end

fprintf('done!\n');

%% Compute gravity terms
fprintf('Computing gravity vector g... ');
g = sym(zeros(3,1));
%TODO: Implement g = ...;

for i=1:3
    fprintf('g%i...', i);
    % pos Jacobian
    I_Js_i = I_Jp{i};
    % mass
    m_i = m{i};
    % cal g
    g = g - I_Js_i' * m_i * I_g_acc;
end

fprintf('done!\n');

%% Compute nonlinear terms
fprintf('Computing coriolis and centrifugal vector b and simplifying... ');
b = sym(zeros(3,1));
%TODO: Implement b = ...;

for i=1:3
    fprintf('b%i...', i);
    % Jacobian
    I_Js_i = I_Jp{i};
    I_Jr_i = I_Jr{i};
    % d Jacobian
    I_dJs_i = dAdt(I_Js_i, q, dq);
    I_dJr_i = dAdt(I_Jr_i, q, dq);
    % ang vel
    I_w_i = I_Jr_i * dq;
    % mass
    m_i = m{i};
    % inertia tensor
    i_I_s_i = k_I_s{i};
    % rot mat
    R_Ii = R_Ik{i};
    % inertia tensor in frame I
    I_I_s_i = R_Ii * i_I_s_i * R_Ii';
    % cal b
    b = b + I_Js_i' * m_i * I_dJs_i * dq;
    b = b + I_Jr_i' * I_I_s_i * I_dJr_i * dq;
    b = b + I_Jr_i' * cross(I_w_i, I_I_s_i * I_w_i);
end

fprintf('done!\n');

%% Store the expressions
eom.M = M;
eom.g = g;
eom.b = b;
end
