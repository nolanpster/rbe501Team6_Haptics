function fig = updateAsymVid(traj,dat1,dat2,dat3,dat4,len,fig)
figure(fig);

    dat1.YData = len;
    dat2.YData = traj.pos(1,:);
    dat3.YData = traj.vel(1,:);
    dat4.YData = traj.acc(1,:);
    pause(0.01)

end 