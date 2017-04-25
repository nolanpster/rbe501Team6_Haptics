% capture a movie of the apparent force
clc
close all
dDel = 0.01;
delta = -1.2:dDel:1.2;
N = length(delta);
w1 = pi + (pi/2)*(delta);
w2 = pi - (pi/2)*(delta);

f =[1,0,0]';
dt = 0.001;
t = 0:dt:5;
traj(N) = asymVibTaskTraj(f,t,w1(end),w2(end));
for i = 1:N-1
    traj(i) =  asymVibTaskTraj(f,t,w1(i),w2(i));
end

% arrow length
len = linspace(-1,1,N);

%%

% x pos
fig = figure('Position', [100, 100, 1049, 895]);

subplot(4,5,1:4)
imshow('posnEq.png')

ax1 = subplot(4,5,[10;15;20]);
dat1 = stem(0,len(1),'g.','Markersize',80);
axis([-0.1 0.1, -1 1])
set(ax1,'YTickLabel','')
set(ax1,'XTickLabel','')
title('Sensation')
ax1Ylim = get(ax1,'Ylim');



ax2 = subplot(4,5,6:9);
dat2 = plot(t,traj(1).pos(1,:));
title('Cartesian Motion')
ylabel('Position','FontSize',12,'FontWeight','bold','Color','r')
ylh = get(gca,'ylabel');
ylp = get(ylh, 'Position');
ext=get(ylh,'Extent');
set(ylh, 'Rotation',0, 'Position',ylp-2*[ext(3) 0 0])

ax2Ylim = get(ax2,'Ylim');
axis([t(1) t(end) ax2Ylim])


% x vel
ax3 = subplot(4,5,11:14);
dat3 = plot(t,traj(1).vel(1,:));
ylabel('Velocity','FontSize',12,'FontWeight','bold','Color','r')
ylh = get(gca,'ylabel');
ylp = get(ylh, 'Position');
ext=get(ylh,'Extent');
set(ylh, 'Rotation',0, 'Position',ylp-2*[ext(3) 0 0])
ax3Ylim = get(ax3,'Ylim');
axis([t(1) t(end)  -ax3Ylim(2) ax3Ylim(2)])

% x acc
ax4 = subplot(4,5,16:19);
dat4 = plot(t,traj(1).acc(1,:));
ylabel('Acceleration','FontSize',12,'FontWeight','bold','Color','r')
ylh = get(gca,'ylabel');
ylp = get(ylh, 'Position');
ext=get(ylh,'Extent');
set(ylh, 'Rotation',0, 'Position',ylp-2*[ext(3) 0 0])
xlabel('sec')
ax4Ylim = get(ax4,'Ylim');
axis([t(1) t(end)  -ax4Ylim(2) ax4Ylim(2)])


linkaxes([ax2 ax3 ax4],'x')
xlim(ax2,[t(1) t(end)])

myMovie(N) = getframe(gcf); % init movie struct


for i = 1:N
    fig = updateAsymVid(traj(i),dat1,dat2,dat3,dat4,len(i),fig);
    myMovie(i) = getframe(gcf);
end

%%
myVideo = VideoWriter('asymVib');
myVideo.FrameRate = 15;
myVideo.Quality = 100;
open(myVideo);
%%
for i = 1:N
    writeVideo(myVideo, myMovie(i));
end

close(myVideo)

