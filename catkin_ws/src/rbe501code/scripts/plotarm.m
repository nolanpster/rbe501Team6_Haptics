%%% _plotarm_
function fig = plotarm(T_comp)
% Plot the ABB IRB-120.
% T = PLOTARM(Q1,Q2,Q3,Q4,Q5,Q6) Plots links and joint positions of
% the ABB IRB-120 given the joint angles. Devnavit-Hartenberg
% parameters are hard-coded into this function.
%
% INPUT:
%   T_comp - [4x4xN] array of composit transforms
%

N = size(T_comp,3);
O = zeros(4,N);
p = [0 0 0 1]';
for i = 1:N
    O(:,i) = T_comp(:,:,i)*p;
end 
Ox = O(1,:);
Oy = O(2,:);
Oz = O(3,:);


fig=figure;
plot3([0 Ox],[0 Oy],[0 Oz],'color',[1 .5 0],'LineWidth',2);
hold on
plot3(0,0,0,'k^','MarkerSize',12,'MarkerFaceColor','k' );
plot3([0 Ox],[0 Oy],[0 Oz],'m.','MarkerSize',20);
xlabel('X [mm]')
ylabel('Y [mm]')
zlabel('Z [mm]')
axis([-.4 .4 -.6 .3 -.5 0])
plotCS(T_comp(:,:,N),fig,0.05);
grid on
legend('Link','Base','Coord. Sys Origin','Tool X-Axis',...
    'Tool Y-Axis','Tool Z-Axis');
end
