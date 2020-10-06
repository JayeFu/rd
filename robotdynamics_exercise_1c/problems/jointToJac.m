function J = jointToJac(q)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector orientation which maps joint
  % velocities to end-effector composite velocities in I frame.
  
  % Position Jacobian matrix
  J_P = jointToPosJac(q);
  % Angle Jacobian matrix
  J_R = jointToRotJac(q);
  
  % Compound J_P and J_R
  J = [J_P; J_R];
end

