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

% Gather all the alphas
alpha1 = -pi/2;
alpha2 = 0;
alpha3 = -pi/2;
alpha4 = pi/2;
alpha5 = -pi/2;
alpha6 = 0;
alphatool = 0;


h = 22;
L = 221;
H = 1104;

% Hardcode in Tw_0
Tw_0 = [sqrt(2)/2 -sqrt(2)/2 0 h; sqrt(2)/2 sqrt(2)/2 0 L; 0 0 1 H; 0 0 0 1];
Rw_0 = Tw_0(1:3,1:3);
dw_0 = Tw_0(1:3,4);

% Gather rotation and d matrices from Tw_tool input
Rw_tool = Tw_tool(1:3,1:3);
dw_tool = Tw_tool(1:3,4);

% Get the values from d
d1 = d(1);
d4 = d(2);
d_tool = d(3);

d6_tool = [0; 0; d_tool]

% Get the values from a
a1 = a(1);
a2 = a(2);

% 1. Solve for Theta 1
d6_tool = [0; 0; d_tool]
R0_6 = Rw_0.' * Rw_tool
%d0_6 = (dw_tool.' * Rw_0) - (dw_0.' * Rw_0) - dw_tool.'*(Rw_0.' *  Rw_tool).'
%d0_6 = (Rw_0.' * dw_tool) - (Rw_0.' * dw_0) - (R0_6.' * d6_tool)
d0_6 = Rw_0 * (dw_tool - dw_0 - Rw_tool.'*d6_tool)
d0_4 = d0_6

x0_4 = d0_4(1)
y0_4 = d0_4(2)
z0_4 = d0_4(3)

theta_1 = atan2(y0_4, x0_4)

% 2. Solve for Theta 3

d1_4 = [cos(theta_1)*(x0_4-a1*cos(theta_1)) - (sin(theta_1)*(y0_4 - a1*sin(theta_1))) ...
        (z0_4 - d1) ...
        -sin(theta_1)*(x0_4 - a1 * cos(theta_1)) - cos(theta_1)*(y0_4 - a1 * sin(theta_1))]

x1_4 = d1_4(1)
y1_4 = d1_4(2)
z1_4 = d1_4(3)

th_3num = (a2+d4)^2 -(x1_4^2+y1_4^2)
th_3den = (x1_4^2 +y1_4^2)-(a2-d4)^2
theta_3 = 2*atan(sqrt(th_3num/th_3den)) % Elbow down
% theta_3 = .395
% 3. Solve for Theta 2

theta_2 = atan2(y1_4, x1_4) - atan2(d4*sin(theta_3), a2+d4*cos(theta_3))

% 4. Solve for Theta 4

%R0_1 = rotz(radtodeg(theta_1)) * rotx(radtodeg(alpha1))
%R1_2 = rotz(radtodeg(theta_2)) * rotx(radtodeg(alpha2))
%R2_3 = rotz(radtodeg(theta_3)) * rotx(radtodeg(alpha3))

R0_1 = [cos(theta_1) 0 -sin(theta_1); sin(theta_1) 0 cos(theta_1); 0 -1 0]
R1_2 = [cos(theta_2) -sin(theta_2) 0; sin(theta_2) cos(theta_2) 0; 0 0 1]
R2_3 = [cos(theta_3) 0 -sin(theta_3); sin(theta_3) 0 cos(theta_3); 0 -1 0]

R0_3 = R0_1 * R1_2 * R2_3
R3_6 = R0_3.' * R0_6

theta_4pos = atan(R3_6(2,3)/R3_6(1,3))
theta_4neg = -theta_4pos

if pi/2 >= theta_4pos >= -pi/2
	theta_4 = theta_4pos
else
	theta_4 = theta_4neg
end

% 5. Solve for Theta 5

theta_5 = atan2(-cos(theta_4)*R3_6(1,3) - sin(theta_4)*R3_6(3,3), R3_6(3,3))

% 6. Solve for Theta 6

theta_6 = atan2(cos(theta_4)*R3_6(2,1) - sin(theta_4)*R3_6(1,1), cos(theta_4)*R3_6(2,2) - sin(theta_4)*R3_6(1,2))

% Compile the thetas into a single vector
theta = [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 0]

end

