function T23 = jointToTransform23(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 3 to frame 2. T_23
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T23 = zeros(4);
  % Rotation angle between frame 3 and frame 2
  t = q(3);
  % Rotation Matrix from frame 3 to frame 2
  C23 = zeros(3);
  C23 = [cos(t), 0, sin(t);
      0, 1, 0;
      -sin(t), 0, cos(t)];
  T23(1:3, 1:3) = C23;
  % Displacement from frame 3 to frame 2
  r23 = [0, 0, 0.270]';
  T23(1:3, 4) = r23;
  % Finally change T(4,4)
  T23(4,4) = 1;
end
