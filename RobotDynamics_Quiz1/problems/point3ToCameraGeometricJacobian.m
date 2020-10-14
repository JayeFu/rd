function [ J_3C_3 ] = point3ToCameraGeometricJacobian(q, params)
  % Inputs:
  %     q: a 3x1 vector of generalized coordinates
  %     params: a struct of parameters
  % Output:
  %     J_3C_3: geometric Jacobian from point O_3 to point C, expressed in
  %     frame {3}
  
  theta = params.theta;
  l31 = params.l31;
  l4 = params.l4;
  l5 = params.l5;
  
  % Joint positions
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);

  % Implement your solution here...
  J_3C_3 = [];
  
  % First calculate rotation matrix
  C_01 = Cy(q1);
  C_12 = Cz(q2);
  C_23 = Cz(q3);
  
  C_13 = C_12*C_23;
  
  % Then ratation axis
  nx = [1, 0, 0]';
  ny = [0, 1, 0]';
  nz = [0, 0, 1]';
  
  n1 = ny;
  n2 = nz;
  n3 = nz;
  
  n1_in_3 = C_13'*n1;
  n2_in_3 = C_23'*n2;
  n3_in_3 = n3;
  
  r_O3C_in_3 = [l31+l4*cos(theta)+l5*sin(theta), l4*sin(theta)-l5*cos(theta), 0]';
  
  J_P = [cross(n1_in_3, r_O3C_in_3), cross(n2_in_3, r_O3C_in_3), cross(n3_in_3, r_O3C_in_3)];
  J_R = [n1_in_3, n2_in_3, n3_in_3];
  
  J_3C_3 = [J_P; J_R];

end