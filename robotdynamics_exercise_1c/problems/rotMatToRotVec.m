function [ phi ] = rotMatToRotVec(C)
% Input: a rotation matrix C
% Output: the rotational vector which describes the rotation C

% Compute the rotional vector
phi = zeros(3,1);

% rotation angle
t = acos(0.5*(C(1,1)+C(2,2)+C(3,3)-1));

% rotation axis
n = 0.5/sin(t)*[C(3,2)-C(2,3),C(1,3)-C(3,1),C(2,1)-C(1,2)]';

phi = t*n;

end
