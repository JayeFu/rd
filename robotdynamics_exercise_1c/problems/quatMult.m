function q_AC = quatMult(q_AB,q_BC)
  % Input: two quaternions to be multiplied
  % Output: output of the multiplication
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  q_AC = zeros(4,1);
  M_l_q_AB = zeros(4);
  % q hat for q_AB
  q_AB_h = q_AB(2:4);
  % anti-symmetric matrix of 
  q_AB_as = [0, -q_AB_h(3), q_AB_h(2);
      q_AB_h(3), 0, -q_AB_h(1);
      -q_AB_h(2), q_AB_h(1), 0];
  M_l_q_AB(1,1) = q_AB(1);
  M_l_q_AB(1,2:4) = -q_AB_h';
  M_l_q_AB(2:4,1) = q_AB_h;
  M_l_q_AB(2:4,2:4) = q_AB(1)*eye(3)+q_AB_as;
  q_AC = M_l_q_AB*q_BC;
end

