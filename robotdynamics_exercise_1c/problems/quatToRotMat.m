function R = quatToRotMat(q)
  % Input: quaternion [w x y z]
  % Output: corresponding rotation matrix
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  R = zeros(3);
  % qh = [x y z]
  qh = q(2:4);
  % anti-symmetric matrix of qh
  qas = [0, -qh(3), qh(2);
      qh(3), 0, -qh(1);
      -qh(2), qh(1), 0];
  R = (2*q(1)^2-1)*eye(3)+2*q(1)*qas+2*qh*qh';
end
