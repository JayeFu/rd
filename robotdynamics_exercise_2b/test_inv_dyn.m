I_r_IE_des = [0.6, 0.4, 0.9]';
eul_IE_des = [-0.2, 0.3, 0.4]';
q = repelem(0.1, 6)';
q_dot = repelem(0.1, 6)';
tau1 = control_inv_dyn_sol(I_r_IE_des, eul_IE_des, q, q_dot)
tau2 = control_inv_dyn(I_r_IE_des, eul_IE_des, q, q_dot)