function T = FKinSpace(M, Slist, thetalist)
% *** CHAPTER 4: FORWARD KINEMATICS ***
% Takes M: the home configuration (position and orientation) of the 
%          end-effector,
%       Slist: The joint screw axes in the space frame when the manipulator
%              is at the home position,
%       thetalist: A list of joint coordinates.
% Returns T in SE(3) representing the end-effector frame, when the joints 
% are at the specified coordinates (i.t.o Space Frame).
% Example Inputs:
% 
% clear; clc;
% M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
% Slist = [[0; 0;  1;  4; 0;    0], ...
%        [0; 0;  0;  0; 1;    0], ...
%        [0; 0; -1; -6; 0; -0.1]];
% thetalist =[pi / 2; 3; pi];
% T = FKinSpace(M, Slist, thetalist)
% 
% Output:
% T =
%   -0.0000    1.0000         0   -5.0000
%    1.0000    0.0000         0    4.0000
%         0         0   -1.0000    1.6858
%         0         0         0    1.0000
% T = FKinSpace(M, Slist, thetalist);

T = M;
for i = size(thetalist): -1 : 1
    T = MatrixExp6(VecTose3(Slist(:, i) * thetalist(i))) * T;
end
end


M = [[1, 0, 0, 0]; [0, 0, -1, -32.32]; [0, 1, 0, -55]; [0, 0, 0, 1]];
Slist = [[-1;0;0;0;-10;-25],[0;-1;0;-20;0;0],[1;0;0;0;-45;42.32]];
thetalistT0 = [deg2rad(60);deg2rad(90);0];

T0 = FKinSpace(M, Slist, thetalistT0);