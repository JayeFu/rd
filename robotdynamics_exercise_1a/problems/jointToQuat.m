function quat = jointToQuat(q)
  % Input: joint angles
  % Output: quaternion representing the orientation of the end-effector
  % q_IE.
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  quat = zeros(4,1);
  % First calculate rotation matrix
  C = jointToRotMat(q);
  quat(1) = sqrt(C(1,1)+C(2,2)+C(3,3)+1);
  quat(2) = sign(C(3,2)-C(2,3))*sqrt(C(1,1)-C(2,2)-C(3,3)+1);
  quat(3) = sign(C(1,3)-C(3,1))*sqrt(C(2,2)-C(3,3)-C(1,1)+1);
  quat(4) = sign(C(2,1)-C(1,2))*sqrt(C(3,3)-C(1,1)-C(2,2)+1);
  quat = 0.5*quat;
end