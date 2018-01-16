%Week 2. Geometry and Kinematics

syms a b real
x = (a + b)^2
expand(x)
y = (a/2)^2/a %a/4

A = [a 0 b; 0 a 0; 0 1 0];
B = [2+a b exp(a); cos(a) 0 0; -a b/a 0];
d = A+B;

%rotation

syms phi theta psi real
R1 = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0;0 0 1];
R2 = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
R3 = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
simplify(R1*R2*R3)

% Determinant = scalar property of square matrices. nxn matrix
% A = [a b; c d]
% det(A) %ad - bc
% y = Ax %Matrix is A. vector y
% 
% Eigenvector = vectors of square matrix that do not change directions
% when multiplied by the matrix
% Eigenvalue = scalars representing how much each eigenvector changes
% in length
% 
% Av = lamba v
% A = matrix, v = eigenvector, lambda = corresp. eigenvalue
% 
% A = [2 1; 1 2]
% 1. Solutions to det(A-lambda*I) = 0. These are 2 distince eigenvalues
% 2. (A - lamba*I)v = 0
