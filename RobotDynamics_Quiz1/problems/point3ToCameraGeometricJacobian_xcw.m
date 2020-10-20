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

  % Implement your solution here...
  
  r_30_3 = [l31+l4*cos(theta)+l5*sin(theta);l4*sin(theta)-l5*cos(theta);0];
  n_1 = [0;1;0];
  n_2 = [0;0;1];
  n_3 = [0;0;1];
  J_3C_3P = [cross(n_1, r_30_3), cross(n_2, r_30_3), cross(n_3, r_30_3)];
  J_3C_3R = zeros(3, 3);
  J_3C_3 = [J_3C_3P;J_3C_3R];

end