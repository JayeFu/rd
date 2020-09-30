function C = Cy(t)
  % Input: scalar value of angle of rotation
  % Output: 3x3 rotation matrix along y-axis
  
  C = [cos(t), 0, sin(t);
      0, 1, 0;
      -sin(t), 0, cos(t)];
end