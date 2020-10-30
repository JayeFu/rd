% generate equations of motion
function eom = generate_eom(gen_cor, kin, dyn, jac)
% By calling:
%   eom = generate_eom(gen_cor, kin, dyn, jac)
% a struct 'eom' is returned that contains the matrices and vectors
% necessary to compute the equations of motion. These are additionally
% converted to matlab scripts.

%% Setup
phi = gen_cor.phi;      % Generalized coordinates (6x1 sym)
dphi = gen_cor.dphi;    % Generalized velocities (6x1 sym)

T_Ik = kin.T_Ik;        % Homogeneous transforms (6x1 cell)->(4x4 sym)
R_Ik = kin.R_Ik;        % Rotation matrices (6x1 cell)->(3x3 sym)

k_I_s = dyn.k_I_s;      % Inertia tensor of body k in frame k (6x1 cell)->(3x3 sym)
m = dyn.m;              % Mass of body k (6x1 cell)->(1x1 double)
I_g_acc = dyn.I_g_acc;  % Gravitational acceleration in inertial frame (3x1 double)
k_r_ks = dyn.k_r_ks;    % CoM location of body k in frame k (6x1 cell)->(3x1 double)

I_Jp_s = jac.I_Jp_s;    % CoM Positional Jacobian in frame I (6x1 cell)->(3x6 sym)
I_Jr = jac.I_Jr;        % CoM Rotational Jacobian in frame I (6x1 cell)->(3x6 sym)

eom.M = sym(zeros(6,6));
eom.g = sym(zeros(6,1));
eom.b = sym(zeros(6,1));
eom.hamiltonian = sym(zeros(1,1));

%% Compute mass matrix
fprintf('Computing mass matrix M... ');
% TODO: Implement M = ...;
M = sym(zeros(6,6));

for i=1:6
    fprintf('M%i...',i);
    % ith link Jacobians
    I_Js_i = I_Jp_s{i};
    I_Jr_i = I_Jr{i};
    % ith link mass
    m_i = m{i};
    % ith link inertia tensor w.r.t. frame i
    i_I_s_i = k_I_s{i};
    % rotation matrix from frame i to frame I
    R_Ii = R_Ik{i};
    % ith link inertia tensor w.r.t. frame I
    I_I_s_i = R_Ii*i_I_s_i*R_Ii';
    % sequentially add mass matrix
    M = M + I_Js_i'*m_i*I_Js_i + I_Jr_i'*I_I_s_i*I_Jr_i;
end

fprintf('done!\n');


%% Compute gravity terms
fprintf('Computing gravity vector g... ');
% TODO: Implement g = ...;
g = sym(zeros(6,1));

for i=1:6
    fprintf('g%i...',i);
    % ith link positiion Jacobian
    I_Js_i = I_Jp_s{i};
    % ith link mass
    m_i = m{i};
    % sequentially add gravity fterm
    g = g - I_Js_i'*m_i*I_g_acc;
end

fprintf('done!\n');


%% Compute nonlinear terms vector
fprintf('Computing coriolis and centrifugal vector b and simplifying... ');
% TODO: Implement b = ...;
b = sym(zeros(6,1));

for i=1:6
    fprintf('b%i...',i);
    % ith link Jacobians
    I_Js_i = I_Jp_s{i};
    I_Jr_i = I_Jr{i};
    % derivaties of Jacobians
    I_dJs_i = simplify(dAdt(I_Js_i, phi, dphi));
    I_dJr_i = simplify(dAdt(I_Jr_i, phi, dphi));
    % ith link angular velocity
    I_w_i = simplify(I_Jr_i*dphi);
    % ith link mass
    m_i = m{i};
    % ith link inertia tensor w.r.t. frame i
    i_I_s_i = k_I_s{i};
    % rotation matrix from frame i to frame I
    R_Ii = R_Ik{i};
    % ith link inertia tensor w.r.t. frame I
    I_I_s_i = simplify(R_Ii*i_I_s_i*R_Ii');
    % sequentially add terms to form b
    b = b + simplify(I_Js_i'*m_i*I_dJs_i*dphi);
    b = b + simplify(I_Jr_i'*I_I_s_i*I_dJr_i*dphi);
    b = b + simplify(I_Jr_i'*cross(I_w_i, I_I_s_i*I_w_i));
end

fprintf('done!\n');


%% Compute energy
fprintf('Computing total energy... ');
% TODO: Implement hamiltonian, enPot, enKin = ...;
hamiltonian = sym(zeros(1,1));
enPot = sym(zeros(1,1));
enKin = sym(zeros(1,1));

for i=1:6
    fprintf('enPot%i...',i);
    % COM location of body i in frame i
    k_r_ks_i = k_r_ks{i};
    k_r_ks_i_expand = [k_r_ks_i; 1];
    % homogenous tf from frame i to I
    T_Ik_i = T_Ik{i};
    % COM location of body i in frame I
    I_r_ks_i_expand = T_Ik_i*k_r_ks_i_expand;
    I_r_ks_i = I_r_ks_i_expand(1:3);
    % ith link mass
    m_i = m{i};
    % sequentially add potential energy
    enPot = enPot - I_r_ks_i'*m_i*I_g_acc;
end

fprintf('enKin...');
enKin = 0.5*dphi'*M*dphi;

fprintf('hamiltonian...');
hamiltonian = enPot+enKin;
fprintf('simplifying hamiltonian...');
hamiltonian = simplify(hamiltonian);

fprintf('done!\n');


%% Generate matlab functions
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');

fprintf('Generating eom scripts... ');
fprintf('M... ');
matlabFunction(M, 'vars', {phi}, 'file', strcat(dpath,'/M_fun'), 'Optimize', false);
fprintf('g... ');
matlabFunction(g, 'vars', {phi}, 'file', strcat(dpath,'/g_fun'), 'Optimize', false);
fprintf('b... ');
matlabFunction(b, 'vars', {phi, dphi}, 'file', strcat(dpath,'/b_fun'), 'Optimize', false);
fprintf('hamiltonian... ');
matlabFunction(hamiltonian, 'vars', {phi, dphi}, 'file', strcat(dpath,'/hamiltonian_fun'), 'Optimize', false);
fprintf('done!\n');


%% Store the expressions
eom.M = M;
eom.g = g;
eom.b = b;
eom.hamiltonian = hamiltonian;
eom.enPot = enPot;
eom.enKin = enKin;

end
