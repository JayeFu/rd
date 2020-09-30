function C = Cz(t)
  % Input: scalar value of angle of rotation
  % Output: 3x3 rotation matrix along z-axis
  
  C = [cos(t), -sin(t), 0;
      sin(t), cos(t), 0;
      0, 0, 1];
end