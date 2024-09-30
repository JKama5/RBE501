%% Davinci Robot HW2 Jack Kamataris

M = [[1, 0, 0, 0]; [0, 0, -1, -32.32]; [0, 1, 0, -55]; [0, 0, 0, 1]];
Slist = [[-1;0;0;0;-10;-25],[0;-1;0;-20;0;0],[1;0;0;0;-45;42.32]];

%% Part 1a

thetalistT0 = [deg2rad(60);deg2rad(90);0];

T0 = FKinSpace(M, Slist, thetalistT0);

%% Part 1b

%% This code does not need to be run, was just for the IK

% % % T = [[0.8660 , 0.25 , -0.4330 , -10.6699];
% % %      [-0.4330 , 0.8080 , -0.3995 ,-65.1557];
% % %      [0.25 , 0.5335 , 0.8080 , 37.4098];
% % %      [0 , 0 , 0 , 1]];
% % % 
% % % thetalist0 = ThetaList ; 
% % % 
% % % eomg = 0.01;
% % % ev = 0.01;
% % % [thetalist,success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev)
% % % 
% % % ThetaList = [thetalist(1);thetalist(2);thetalist(3)];
% % % G = FKinSpace(M, Slist, ThetaList)
% % % ThetaList = wrapTo2Pi(ThetaList)

%% This code above does not need to be run but was for the IK 

thetalistT1 = [0;0;0];
thetalistT2 = [1.5708; 0.7854; -0.7854];
thetalistT3 = [2.0948; 5.7596; 1.0473];

% change thetalistTx to be thetalistTx where x is the # transform you want
Tx = FKinSpace(M, Slist, thetalistT3);


%% Question 2e

Blist =[[0,1,0,1,0,1,0];
        [0,0,0,0,0,0,0];
        [1,0,1,0,1,0,1];
        [0,0,0,0,0,0,0];
        [0,-0.95,0,-0.55,0,-0.15,0];
        [0,0,0,0,0,0,0]];

deg30 = deg2rad(30);
Bthetalist = [deg30;deg30;deg30;deg30;deg30;deg30;deg30];
Jb = JacobianBody(Blist, Bthetalist);

Torque = transpose(Jb) * [1;1;1;1;1;1];



    