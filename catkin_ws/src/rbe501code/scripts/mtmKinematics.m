%% MTM Kinematics
%%
% load DH params and subscribe to MTM positions

rosSubscribers
load mtmDH
qVec = sym(zeros(7,1));
thetaZ = sym(thetaZ);
for i = 1:7
    thetaZ(i) = sym(thetaZ(i)+['q',num2str(i)]);
    qVec(i) = sym(['q',num2str(i)]);
end 
sym(pi);
fNames = {'1','2', '3', '4', '6', '6', '7'};
%%
T = dh2mat(thetaZ,dZ,aX,alphaX);

%%
% I comput the transformation matricies to each of the joints, and the
% COM positions in the base frame.
fprintf('Intermediate frame transformationss are:\n')
T_0_7 = eye(4);
for i = 1:length(fNames)-1
    % Display intermediate transforms, also build composite transform.
    % T(:,:,i) = subs(T(:,:,i),{1.0, -1.0},{1, -1});
    T_0_7 = simplify(T_0_7*T(:,:,i));
    fprintf(['From frame ',fNames{i},' to ',fNames{i+1},':\n']);
    disp(T(:,:,i))
end 
%%
% The composit transform from the robot base frame,R, to the was also
% calculated above.
fprintf('Composite Transformation from frame 0 to 3 :\n')
T_links = sym(zeros(4,4,7));
%%
% First calculate composite transformations
%%
% 
T_0_1 = simplify(T(:,:,1));
T_links(:,:,1) = T_0_1; %store for later.
fprintf('Composite Transformation from frame 0 to 1 :\n')
disp(T_0_1);
%%
% 
T_0_2 = simplify(T(:,:,1)*T(:,:,2));
T_links(:,:,2) = T_0_2; %store for later.
fprintf('Composite Transformation from frame 0 to 2 :\n')
disp(T_0_2);
%%
%
T_0_3 = simplify(T(:,:,1)*T(:,:,2)*T(:,:,3));
T_links(:,:,3) = T_0_3; %store for later.
fprintf('Composite Transformation from frame 0 to 3 :\n')
disp(T_0_3);
%%
% 
T_0_4 = simplify(T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4));
T_links(:,:,4) = T_0_4; %store for later.
fprintf('Composite Transformation from frame 0 to 4 :\n')
disp(T_0_4);
%%
T_0_5 = simplify(T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5));
T_links(:,:,5) = T_0_5; %store for later.
fprintf('Composite Transformation from frame 0 to 5 :\n')
disp(T_0_5);
%%ReadAllBoards: handle for port 0 is NULL
T_0_6 = simplify(T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6));
T_links(:,:,6) = T_0_6; %store for later.
fprintf('Composite Transformation from frame 0 to 6 :\n')
disp(T_0_6);
%%
T_0_7 = simplify(T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*...
    T(:,:,7));
T_links(:,:,7) = T_0_7; %store for later.
fprintf('Composite Transformation from frame 0 to 7 :\n')
disp(T_0_7);
%%
% Need to do next transformation
%% Problem 2
% For the velocity kinematics I extract the rotation axes of each 
% joint, expressed in the reference/robot
% frame, from the 3rd column of the transformation matricies to each 
% joint. 
z0 = simplify(T_0_1(1:3,3));
z1 = simplify(T_0_2(1:3,3));
z2 = simplify(T_0_3(1:3,3));
%%
% Note that all three of these are unit vectors along $\hat{z}_0$
disp('Concatenated rotation axes')
disp([z0,z1,z2])
%%
% Then I extract the positions of the end of each link tips and
% the COMs in the reference frame.
p0 = zeros(3,1);
p1 = simplify(T_0_1(1:3,4));
p2 = simplify(T_0_2(1:3,4));
p3 = simplify(T_0_3(1:3,4));
p4 = simplify(T_0_4(1:3,4));
p5 = simplify(T_0_5(1:3,4));
p6 = simplify(T_0_6(1:3,4));
p7 = simplify(T_0_7(1:3,4));

%%
% I calculate the linear jacobiian for the 3rd link
J_3 = simplify(jacobian(p3,qVec(1:3)));

fprintf('Jacobian of Position of Joint 4\n\r')
pretty(J_3);
invJ_3 = inv(J_3);
%%
% Check cartesian position of MTM gripper
% Only use current 'q' values 1:7 because I think the 8th is the pincer
% status
qCurr = receive(jointQSub,10);
T_linksCurrent = double(subs(T_links,qVec,qCurr.Position(1:7)));
plotarm(T_linksCurrent)

%%
% Create task space asymmetric oscilation offests
% See example script configAsymWave.m and asymVibTaskTraj.m for more
% details.
delta = 1;
dt = 0.01;
w1 = pi + (pi/2)*(delta);
w2 = pi - (pi/2)*(delta);
f =[1,1,0]';
t = 0:dt:3;
traj = asymVibTaskTraj(f,t,w1,w2);

%%
% Need to calculate IK. We sould calculate IK to get position trajectory,
% and compare with using integrated InvVelKin
%%
% Inverse Velocity Kinematics
load('joint3Jac.mat')
qCurr = receive(jointQSub,10);
invJ_3curr = double(subs(invJ_3,qVec,qCurr.Position(1:7)));
qVel = invJ_3curr*traj.vel;
figure
subplot(3,1,1)
plot(t,qVel(1,:))
title 'Necessary Joint Velocities'
ylabel('q1 vel')
subplot(3,1,2)
plot(t,qVel(2,:))
ylabel('q2 vel')
subplot(3,1,3)
plot(t,qVel(3,:))
ylabel('q3 vel')
% "Integral" of joint velocity for joint positions
qPos_offset = cumsum(qVel,2)*dt;
qPos = qCurr.Position(1:3)+qPos_offset;
figure
subplot(3,1,1)
plot(t,qPos(1,:))
title 'Necessary Joint Position Trajectories'
ylabel('q1 pos')
subplot(3,1,2)
plot(t,qPos(2,:))
ylabel('q2 pos')
subplot(3,1,3)
plot(t,qPos(3,:))
ylabel('q3 pos')