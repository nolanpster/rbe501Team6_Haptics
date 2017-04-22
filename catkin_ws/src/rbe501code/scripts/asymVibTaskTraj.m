function traj = asymVibTaskTraj(dirVec,t,w1,w2)
% Generates a task space trajectory for asymmetric vibration
% Inputs:
%  dirVec  - 3x1 direction vector [x;y;z];
%  t       - time vector [0:deltaT:tf] tf \in integer (for  now)
N = length(t);
asym = zeros(N,1);
x = 2*pi*mod(t,1);
% identify where signal should switch (when x*pi/w1 = pi)
x1 = x<w1;
x2 = x>=w1;
phi = pi*(1-w1/w2); % phase offset (x*pi/w1 = w1*pi/w2 + phi, where x = w1)

% Position oscilation components
asym(x1) = -cos(x(x1)*pi/w1);
asym(x2) = -cos(x(x2)*pi/w2 + phi);

% splitting this into x-y-z components
mat_1 = kron(dirVec,ones(1,N));
mat_2 = kron(asym,ones(1,3))';
zero3 = zeros(3,1);
traj.pos=mat_1.*mat_2;
traj.vel = [zero3,diff(traj.pos,1,2)];
traj.acc = [zero3,diff(traj.vel,1,2)];

end 