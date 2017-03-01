function H = fwd_kin(theta)
%% Task 1
% theta  -> rotates about the local z axis
% alpha  -> rotates about the local x axis
% Rotation matrix is build as succesive rotation of alpha about the x-axis 
% followed by a rotation of theta about the z-axis
% End effector position is mapped back from the end effector position to
% intertial coordinates

clc

if length(theta) ~= 6
    error('Wrong input. Please check number of joints')
end
                    
Rx = @(angle) [1    0           0; ...
               0    cos(angle)  sin(angle); ...
               0    -sin(angle) cos(angle)];
 
Rz = @(angle) [cos(angle)   sin(angle)  0;...
               -sin(angle)  cos(angle)  0;...
               0            0           1];

% Creates list of parameters
l1 = 0.213278;
l2 = 0.3;
l3 = 0.225;
l4 = 0.075;

% Rotations about the x axis
alpha = [0 pi/2 0 pi/2 -pi/2 -pi/2];
theta(2) = pi/2 + theta(2); %To be aligned with the 2nd frame

% Define the origin of the coordinate frames respect to its predecesor
p_i{1} = [0 0 l1]';   % Defined respect to Inertial frame
p_i{2} = [0 0 0]';    % Defined respect to frame 1
p_i{3} = [l2 0 0]';   % Defined respect to frame 2
p_i{4} = [0 -l3 0]';  % Defined respect to frame 3
p_i{5} = [0 0 0]';    % Defined respect to frame 4
p_i{6} = [0 -l4 0]';  % Defined respect to frame 5

R = diag(ones(3,1));
p = zeros(3,1);

for i=1  : length(theta)
    R_i{i}  = Rz(theta(i))*Rx(alpha(i));   
    R       = R * R_i{i}';
end

p = [0 0 0]';
for i = length(theta) : -1 : 1
    p = p_i{i} + R_i{i}'* p;
end

% plot
close all
figure(1)
point_i = zeros(3,1);
Rot_i = diag(ones(1,3));
point_0 = zeros(3,1);
for i = 1 : length(theta)
    
    point_i = point_0 + Rot_i*p_i{i} 
    Rot_i   = Rot_i*R_i{i}';
    plot3([point_0(1) point_i(1)],[point_0(2) point_i(2)],[point_0(3) point_i(3)],'Marker','o','LineWidth',2)
    point_0 = point_i;
    hold on
end
set(gca,'xlim',[-1 1], 'ylim',[-1 1], 'zlim', [-1 1])
axis equal
grid on

% Homogeneous transformation
% Rotation matrix maps vectors expressed in the end effector frame to the
% inertial frame
% Position vector describes end effector position with respect to Inertial
% frame expressed in Inertial frame coordinates
H = [R,p;zeros(1,3),1];

end