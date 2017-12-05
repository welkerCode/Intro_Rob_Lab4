%IKNELBOW Inverse Kinematics in elbow
%
%  theta = ikinelbow(a,d,Tw_tool) returns an array of joint angles base on an array of nonzero a and d DH parameters.
%  a       = vector of nonzero distances along the x_i axis from reference frame to the tool frame
%  d       = vector of nonzero distances 
%  Tw_tool = the tool frame
%  
%  theta is the 6 joint angles that need to be returned
% a and d are arrays containing the non-zero DH length parameters, therefore, 2 for the a's (1, 2,) and 3 for the d's (1, 4, tool)

% Must be: Lefty, Elbow Down and No-Flip (joint 5 is in quad 1 or 4)
% Must handle exceptional cases (like when the desired position is outside of the workspace

function theta = ikinelbow(a,d,Tw_tool)

