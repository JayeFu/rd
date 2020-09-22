function q = rotMatToQuat(R)
  % Input: rotation matrix
  % Output: corresponding quaternion [w x y z]
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  q = zeros(4,1);
  q(1) = sqrt(R(1,1)+R(2,2)+R(3,3)+1);
  q(2) = sign(R(3,2)-R(2,3))*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
  q(3) = sign(R(1,3)-R(3,1))*sqrt(R(2,2)-R(3,3)-R(1,1)+1);
  q(4) = sign(R(2,1)-R(1,2))*sqrt(R(3,3)-R(1,1)-R(2,2)+1);
  q = 0.5*q;
end