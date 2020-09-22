function T56 = jointToTransform56(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 6 to frame 5. T_56
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T56 = zeros(4);
  % Rotation angle between frame 6 and frame 5
  t = q(6);
  % Rotation Matrix from frame 6 to frame 5
  C56 = zeros(3);
  C56 = [1, 0, 0;
      0, cos(t), -sin(t);
      0, sin(t), cos(t)];
  T56(1:3, 1:3) = C56;
  % Displacement from frame 6 to frame 5
  r56 = [0.072, 0, 0]';
  T56(1:3, 4) = r56;
  % Finally change T(4,4)
  T56(4,4) = 1;
end
