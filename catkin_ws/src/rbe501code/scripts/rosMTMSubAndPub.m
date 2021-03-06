% Subscribe to ROS node and grab single message
% While running dvrk_kinematics test_mtm_logic.launch
clear
if ~robotics.ros.internal.Global.isNodeActive
    rosinit
end
%% Task Space Position
taskPoseSub = rossubscriber('/dvrk_mtm/cartesian_pose_current',...
    'geometry_msgs/Pose');
taskPos = receive(taskPoseSub,10)


%% Task Space Position
jointQSub = rossubscriber('/dvrk_mtm/joint_states', ...
    'sensor_msgs/JointState');
qCurr = receive(jointQSub,10)
%%
% Symbolic joint variables for substituting into jacobian
qVec = sym(zeros(7,1));
thetaZ = sym(zeros(7,1));
for i = 1:7
    thetaZ(i) = sym(thetaZ(i)+['q',num2str(i)]);
    qVec(i) = sym(['q',num2str(i)]);
end 
sym(pi);
%%
% Generate Cartesian Trajectory 
delta = -1; % adjust this 
dt = 0.01;
w1 = pi + (pi/2)*(delta);
w2 = pi - (pi/2)*(delta);
f =[0.01,0.01,0]';
t = 0:dt:3;
N = length(t);
traj = asymVibTaskTraj(f,t,w1,w2);
%%
% Generate joint velocity and position commands
load joint3Jac.mat
% velocity
qCurr = receive(jointQSub,10);
invJ_3curr = double(subs(invJ_3,qVec,qCurr.Position(1:7)));
qVel = invJ_3curr*traj.vel;
qVel =[qVel;zeros(5,N)];
% "integrate" velocity to get position (this method is a rough
% approximation
qPos_offset = cumsum(qVel(1:3,:),2)*dt;
qPos = qCurr.Position(1:3)+qPos_offset;
qPos = [qPos;zeros(5,N)];
%%
% MTM Control mode Publisher
mtmModePub = rospublisher('/dvrk_mtm/control_mode');
mtmModeCmd = rosmessage(mtmModePub);
MODE_RESET = 0;
MODE_MANUAL = 1;
MODE_HOLD = 2;
MODE_CLUTCH = 3;
MODE_TELEOP = 4;
mtmModeCmd.Data = MODE_TELEOP;
send(mtmModePub,mtmModeCmd);

%%
% not sure if this is the right topic? There is something else publishing
% to this, and the robot isn't listening to it so I'm not sure...
qPosPub = rospublisher('/dvrk_mtm/matlab_joint_command',...
    'sensor_msgs/JointState');
qPosMsg(N) = rosmessage(qPosPub);
for i = 1:N
    qPosMsg(i) = rosmessage(qPosPub);
    qPosMsg(i).Position = qPos(:,i);
    qPosMsg(i).Velocity = qVel(:,i);
    send(qPosPub,qPosMsg(i));
    pause(0.005)
end 
%%
% % Cartesian pose cmd
% taskPosPub = rospublisher('/dvrk_mtm/cartesian_pose_command',...
%     'geometry_msgs/Pose');
% taskPosMsg(N) = rosmessage(taskPosPub);
% %%
% for i = 1:N
%     taskPosMsg(i) = rosmessage(taskPosPub);
%     taskPosMsg(i).Position.X = traj.pos(1,i);
%     taskPosMsg(i).Position.Y = traj.pos(2,i);
%     taskPosMsg(i).Position.Z = traj.pos(3,i);
%     send(taskPosPub,taskPosMsg(i));
%     pause(0.005)
% end 