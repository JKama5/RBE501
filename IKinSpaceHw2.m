function [thetalist, success] ...
         = IKinSpace(Slist, M, T, thetalist0, eomg, ev)
% *** CHAPTER 6: INVERSE KINEMATICS ***
% Takes Slist: The joint screw axes in the space frame when the manipulator
%              is at the home position, in the format of a matrix with the
%              screw axes as the columns,
%       M: The home configuration of the end-effector,
%       T: The desired end-effector configuration Tsd,
%       thetalist0: An initial guess of joint angles that are close to 
%                   satisfying Tsd,
%       eomg: A small positive tolerance on the end-effector orientation 
%             error. The returned joint angles must give an end-effector 
%             orientation error less than eomg,
%       ev: A small positive tolerance on the end-effector linear position 
%           error. The returned joint angles must give an end-effector 
%           position error less than ev.
% Returns thetalist: Joint angles that achieve T within the specified 
%                    tolerances,
%         success: A logical value where TRUE means that the function found
%                  a solution and FALSE means that it ran through the set 
%                  number of maximum iterations without finding a solution
%                  within the tolerances eomg and ev.
% Uses an iterative Newton-Raphson root-finding method.
% The maximum number of iterations before the algorithm is terminated has 
% been hardcoded in as a variable called maxiterations. It is set to 20 at 
% the start of the function, but can be changed if needed.  
% Example Inputs:
% 
%clear; clc;
% Slist = [[0; 0;  1;  4; 0;    0], ...
%        [0; 0;  0;  0; 1;    0], ...
%        [0; 0; -1; -6; 0; -0.1]];
% M = [[-1, 0, 0, 0]; [0, 1, 0, 6]; [0, 0, -1, 2]; [0, 0, 0, 1]];
% T = [[0, 1, 0, -5]; [1, 0, 0, 4]; [0, 0, -1, 1.6858]; [0, 0, 0, 1]];
% thetalist0 = [1.5; 2.5; 3];
% eomg = 0.01;
% ev = 0.001;
% [thetalist, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev)

% Output:
% thetalist =
%    1.5707
%    2.9997
%    3.1415
% success =
%     1

thetalist = thetalist0;
i = 0;
maxiterations = 20;
Tsb = FKinSpace(M, Slist, thetalist);
Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
err = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
while err && i < maxiterations
    thetalist = thetalist + pinv(JacobianSpace(Slist, thetalist)) * Vs;
    i = i + 1;
    Tsb = FKinSpace(M, Slist, thetalist);
    Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
    err = norm(Vs(1: 3)) > eomg || norm(Vs(4: 6)) > ev;
end
success = ~ err;
end

%% T2 and T3 %%

M = [[1, 0, 0, 0]; [0, 0, -1, -32.32]; [0, 1, 0, -55]; [0, 0, 0, 1]];

Slist = [[-1;0;0;0;-10;-25],[0;-1;0;-20;0;0],[1;0;0;0;-45;42.32]];

T2 = [[0.7071 , -0.5000, -0.5000 , 27.6777];
     [0.7071 , 0.5000 , 0.5000 ,-82.6777];
     [0.25 , 0.5335 , 0.8080 , 27.3200];
     [0 , 0 , 0 , 1.000]];

T3 = [[0.8660 , 0.2500 , -0.4330 , -10.6699];
     [-0.4330 , 0.8080 , -0.3995 ,-65.1557];
     [0.2500 , 0.5335 , 0.8080 , 37.4098];
     [0 , 0 , 0 , 1.000]];

eomg = 0.01;

ev = 0.01;

thetalistT2 = [1.5708; 0.7854; -0.7854]; % the solved values
thetalistT3 = [2.0948; 5.7596; 1.0473]; % the solved values

 % Replace T3 with T2 and thetalistT3 with thetalistT2 to converge for T2
[thetalist, success] = IKinSpace(Slist,M,T3,thetalistT3,eomg,ev);


