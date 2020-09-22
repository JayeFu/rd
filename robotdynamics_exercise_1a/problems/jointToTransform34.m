function T34 = jointToTransform34(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 4 to frame 3. T_34
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T34 = zeros(4);
  % Rotation angle between frame 4 and frame 3
  t = q(4);
  % Rotation Matrix from frame 4 to frame 3
  C34 = zeros(3);
  C34 = [1, 0, 0;
      0, cos(t), -sin(t);
      0, sin(t), cos(t)];
  T34(1:3, 1:3) = C34;
  % Displacement from frame 4 to frame 3
  r34 = [0.134, 0, 0.070]';
  T34(1:3, 4) = r34;
  % Finally change T(4,4)
  T34(4,4) = 1;
end

