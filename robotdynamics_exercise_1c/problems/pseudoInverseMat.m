function [ pinvA ] = pseudoInverseMat(A, lambda)
% Input: Any m-by-n matrix, and a damping factor.
% Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula

% Get the number of rows (m) and columns (n) of A
[m, n] = size(A);

% TODO: complete the computation of the pseudo-inverse.
% Hint: How should we account for both left and right pseudo-inverse forms?
pinvA = zeros(n, m);

% Whether left inverse or right inverse
if m>=n % left inverse
    pinvA = (A'*A+lambda^2*eye(n))\A';
elseif m<=n
    pinvA = A'/(A*A'+lambda^2*eye(m));
else
    disp('Sth goes wrong')
end
end
