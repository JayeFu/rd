function T01 = jointToTransform01(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 1 to frame 0. T_01
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T01 = zeros(4);
  % Rotation angle between frame 1 and frame 0
  t = q(1);
  % Rotation Matrix from frame 1 to frame 0
  C01 = zeros(3);
  C01 = [cos(t), -sin(t), 0;
      sin(t), cos(t), 0;
      0, 0, 1];
  T01(1:3, 1:3) = C01;
  % Displacement from frame 1 to frame 0
  r01 = [0, 0, 0.145]';
  T01(1:3, 4) = r01;
  % Finally change T(4,4)
  T01(4,4) = 1;
end