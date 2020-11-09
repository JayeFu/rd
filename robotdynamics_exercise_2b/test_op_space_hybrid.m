I_r_IE_des = [0.6, 0.4, 0.9]';
eul_IE_des = [-0.2, 0.3, 0.4]';
q = repelem(0.1, 6)';
dq = repelem(0.1, 6)';
I_F_E_x = 5.0;

control_op_space_hybrid(I_r_IE_des, eul_IE_des, q, dq, I_F_E_x)
control_op_space_hybrid_sol(I_r_IE_des, eul_IE_des, q, dq, I_F_E_x)