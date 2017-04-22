
% How steep the asym-vib is (can be between -1 and 1 - actually can be any
% value, but it works best between -1 and 1)
delta = -1;

w1 = pi + (pi/2)*(delta);
w2 = pi - (pi/2)*(delta);
%w1 = pi + (pi/2)*(delta);
%w2 = pi - (pi/2)*(1-delta);
f =[1,0.3,.7]';
t = -1:0.001:3;

traj = asymVibTaskTraj(f,t,w1,w2);
figure
% x pos
ax1 = subplot(3,3,1);
plot(t,traj.pos(1,:))
ylabel('Posn')
title('X')
% y pos
ax2 = subplot(3,3,2);
plot(t,traj.pos(2,:))
ylabel('Posn')
title('Y')
% z pos
ax3 = subplot(3,3,3);
plot(t,traj.pos(3,:))
ylabel('Posn')
title('Z')

% x vel
ax4 = subplot(3,3,4);
plot(t,traj.vel(1,:))
ylabel('Vel')
% y vel
ax5 = subplot(3,3,5);
plot(t,traj.vel(2,:))
ylabel('Vel')

% z vel
ax6 = subplot(3,3,6);
plot(t,traj.vel(3,:))
ylabel('Vel')


% x acc
ax7 = subplot(3,3,7);
plot(t,traj.acc(1,:))
ylabel('acc')
xlabel('sec')
% y acc
ax8 = subplot(3,3,8);
plot(t,traj.acc(2,:))
ylabel('acc')
xlabel('sec')
% z acc
ax9 = subplot(3,3,9);
plot(t,traj.acc(3,:))
ylabel('acc')
xlabel('sec')
linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8 ax9],'x')
xlim(ax1,[t(1) t(end)])