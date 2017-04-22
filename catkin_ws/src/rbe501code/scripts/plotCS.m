%%% _plotCS_
function hand = plotCS(T,fig,axLen)
x0 = [axLen 0 0 1]';
y0 = [0 axLen 0 1]';
z0 = [0 0 axLen 1]';
o0 = [0 0 0 1]';
pts = T*[x0,y0,z0,o0];
x = pts(1:3,1);
y = pts(1:3,2);
z = pts(1:3,3);
o = pts(1:3,4);
xHat = [o x];
yHat = [o y];
zHat = [o z];

hand = cell(3,1);
figure(fig);
hold on
hand{1} = line(xHat(1,:),xHat(2,:),xHat(3,:),'color','r','LineWidth',2);
hand{2} = line(yHat(1,:),yHat(2,:),yHat(3,:),'color','g','LineWidth',2);
hand{3} = line(zHat(1,:),zHat(2,:),zHat(3,:),'color','b','LineWidth',2);
end 
