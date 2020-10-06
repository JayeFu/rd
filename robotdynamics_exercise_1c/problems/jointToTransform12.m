function T12 = jointToTransform12(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 2 to frame 1. T_12
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T12 = zeros(4);
  % Rotation angle between frame 2 and frame 1
  t = q(2);
  % Rotation Matrix from frame 2 to frame 1
  C12 = zeros(3);
  C12 = [cos(t), 0, sin(t);
      0, 1, 0;
      -sin(t), 0, cos(t)];
  T12(1:3, 1:3) = C12;
  % Displacement from frame 2 to frame 1
  r12 = [0, 0, 0.145]';
  T12(1:3, 4) = r12;
  % Finally change T(4,4)
  T12(4,4) = 1;
end