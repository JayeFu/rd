function T45 = jointToTransform45(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 5 to frame 4. T_45
  
  % PLACEHOLDER FOR OUTPUT -> REPLACE WITH SOLUTION
  T45 = zeros(4);
  % Rotation angle between frame 5 and frame 4
  t = q(5);
  % Rotation Matrix from frame 5 to frame 4
  C45 = zeros(3);
  C45 = [cos(t), 0, sin(t);
      0, 1, 0;
      -sin(t), 0, cos(t)];
  T45(1:3, 1:3) = C45;
  % Displacement from frame 5 to frame 4
  r45 = [0.168, 0, 0]';
  T45(1:3, 4) = r45;
  % Finally change T(4,4)
  T45(4,4) = 1;
end

