%FKINE Forward kinematics from DH parameter vectors.
%
%	T = fkine(a,d,alpha,theta) returns the homogenous
%       transformation matrix based on vectors of DH parameters 
%	a, d, alpha, and theta.
%
%	a     = vector of distances from z_i-1 to z_i along x_i.
%	d     = vector of distances from x_i-1 to x_i along z_i-1.
%	alpha = vector of skew angles about x_i.
%	theta = vector of rotation angles about z_i-1.
%	T     = 4x4 homegenous matrix.

function T = fkine(a,d,alpha,theta)
L = 278;
h = 64;
H = 1104;
Tw_bl = [sqrt(2)/2  sqrt(2)/2 0  L; ...
         -sqrt(2)/2 sqrt(2)/2 0 -h; ...
         0          0         1  H; ...
         0          0         0  1];
T = Tw_bl; 
%T = eye(4);
for i = 1:length(a)
	T = T * linktrans(a(i), d(i), alpha(i), theta(i));
end
