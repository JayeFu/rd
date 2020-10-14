function [ T_IE ] = jointToEndeffectorPose( q, params )
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters

  % Link lengths (meters)
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l31 = params.l31;
  l32 = params.l32;

  % Joint positions
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);
    
  % Implement your solution here ...
  T_IE = []; 
  
  % Calculate subsequent transform matrix
  c_base = [0, 0, 0, 1];
  
  C_I0 = [0, 0, 1;
      1, 0, 0;
      0, 1, 0];
  r_I0 = [0, 0, l0]';
  T_I0 = [C_I0, r_I0;
      c_base];
  
  C_01 = Cy(q1);
  r_01 = [0, l1, 0]';
  T_01 = [C_01, r_01;
      c_base];
  
  C_12 = Cz(q2);
  r_12 = [0, 0, 0]';
  T_12 = [C_12, r_12;
      c_base];
  
  C_23 = Cz(q3);
  r_23 = [l2, 0, 0]';
  T_23 = [C_23, r_23;
      c_base];
  
  C_3E = eye(3);
  r_3E = [l31+l32, 0, 0]';
  T_3E = [C_3E, r_3E;
      c_base];
  
  T_IE = T_I0*T_01*T_12*T_23*T_3E;
  
end