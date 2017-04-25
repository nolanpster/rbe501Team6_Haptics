close all
% How steep the asym-vib is (can be between -1 and 1 - actually can be any
% value, but it works best between -1 and 1)
delta = 1;

w1 = pi + (pi/2)*(delta);
w2 = pi - (pi/2)*(delta);
%w1 = pi + (pi/2)*(delta);
%w2 = pi - (pi/2)*(1-delta);
f =[1,0.3,.7]';
t = 1:0.001:2;

traj = asymVibTaskTraj(f,t,w1,w2);
fig = figure('Position', [100, 100, 1049, 895]);
vecStr = [num2str(f(1)),',',num2str(f(2)),',',num2str(f(3))];
descr = {sprintf('Cartesian Incremental Motion Coresponding to $\\vec{F} = [%s]^T$',vecStr)};
subplot(4,3,1:3)
text(0.1,0,descr,'FontSize',18,'FontWeight','bold','Interpreter','Latex'); axis off

% x pos
ax1 = subplot(4,3,4);
plot(t,traj.pos(1,:))
ylim([0 max(f)+1])
ylabel('Posn')
title('X')
% y pos
ax2 = subplot(4,3,5);
plot(t,traj.pos(2,:))
ylabel('Posn')
ylim([0 max(f)+1])
title('Y')
% z pos
ax3 = subplot(4,3,6);
plot(t,traj.pos(3,:))
ylabel('Posn')
ylim([0 max(f)+1])
title('Z')

% x vel
ax4 = subplot(4,3,7);
plot(t,traj.vel(1,:))
ylabel('Vel')
% y vel
ax5 = subplot(4,3,8);
plot(t,traj.vel(2,:))
ylabel('Vel')

% z vel
ax6 = subplot(4,3,9);
plot(t,traj.vel(3,:))
ylabel('Vel')


% x acc
ax7 = subplot(4,3,10);
plot(t,traj.acc(1,:))
ylabel('acc')
xlabel('sec')
% y acc
ax8 = subplot(4,3,11);
plot(t,traj.acc(2,:))
ylabel('acc')
xlabel('sec')
% z acc
ax9 = subplot(4,3,12);
plot(t,traj.acc(3,:))
ylabel('acc')
xlabel('sec')
linkaxes([ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8 ax9],'x')
xlim(ax1,[t(1) t(end)])