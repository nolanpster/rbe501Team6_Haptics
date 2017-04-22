% Subscribe to ROS node and grab single message
% While running dvrk_kinematics test_mtm_logic.launch

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