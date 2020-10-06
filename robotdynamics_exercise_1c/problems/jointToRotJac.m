function J_R = jointToRotJac(q)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector orientation which maps joint
  % velocities to end-effector angular velocities in I frame.

  % Compute the rotational jacobian.
  J_R = zeros(3, 6);
  
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
  
  J_R = [I_n_1, I_n_2, I_n_3, I_n_4, I_n_5, I_n_6];

end
