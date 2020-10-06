function J_P = jointToPosJac(q)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector translation which maps joint
  % velocities to end-effector linear velocities in I frame.

  % Compute the translational jacobian.
  J_P = zeros(3, 6);

  % first calculate roation matrix from frame I to other frames
  C_I0 = eye(3);
  C_I1 = C_I0*Cz(q(1));
  C_I2 = C_I1*Cy(q(2));
  C_I3 = C_I2*Cy(q(3));
  C_I4 = C_I3*Cx(q(4));
  C_I5 = C_I4*Cy(q(5));
  C_I6 = C_I5*Cx(q(6));
  C_IE = C_I6*eye(3);
  
  % define unit axis vector
  ex = [1,0,0]';
  ey = [0,1,0]';
  ez = [0,0,1]';
  
  % then calculate rotation axis-vector
  I_n_1 = C_I0*ez;
  I_n_2 = C_I1*ey;
  I_n_3 = C_I2*ey;
  I_n_4 = C_I3*ex;
  I_n_5 = C_I4*ey;
  I_n_6 = C_I5*ex;
  I_n_E = C_I6*ex;
  
  % iniversely calculate I_r_n(n+1)
  I_r_6E = C_I6*zeros(3,1);
  I_r_5E = C_I5*0.072*ex + I_r_6E;
  I_r_4E = C_I4*0.168*ex + I_r_5E;
  I_r_3E = C_I3*(0.134*ex+0.070*ez) +I_r_4E;
  I_r_2E = C_I2*0.270*ez + I_r_3E;
  I_r_1E = C_I1*0.145*ez + I_r_2E;
  
  % finally integrate all cross products into J_P
  J_P = [cross(I_n_1,I_r_1E), cross(I_n_2,I_r_2E), cross(I_n_3,I_r_3E), ...
      cross(I_n_4,I_r_4E), cross(I_n_5,I_r_5E), cross(I_n_6,I_r_6E)];
end