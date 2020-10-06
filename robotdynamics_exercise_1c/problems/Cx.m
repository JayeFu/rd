function C = Cx(t)
  % Input: scalar value of angle of rotation
  % Output: 3x3 rotation matrix along x-axis
  
  C = [1, 0, 0;
      0, cos(t), -sin(t);
      0, sin(t), cos(t)];
end

