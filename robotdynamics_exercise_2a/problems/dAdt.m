function dA = dAdt(A, q, dq)
%dAdt calculate derivative of matrix A given generalized coordinates and
%their derivatives

dA = sym(zeros(size(A)));
for i=1:size(A,1)
    for j=1:size(A,2)
        dA(i,j) = jacobian(A(i,j),q)*dq
    end
end

end

