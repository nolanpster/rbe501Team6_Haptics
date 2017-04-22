%% Poulin: RBE 501 HW 4
% Due Apr. 4. 2017: 11:59Pm
%%
% load DH params and subscribe to MTM positions
load mtmDH
rosSubscribers

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
%%
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
% I calculate the jacobian for the 3rd link
J_3 = simplify(jacobian(p3,qVec(1:3)));

fprintf('Jacobian of Position of Joint 4\n\r')
pretty(J_3);
%%
% Check cartesian position of MTM gripper
qCurr = receive(jointQSub,10);
T_linksCurrent = double(subs(T_links,qVec,qCurr.Position(1:7)));
plotarm(T_linksCurrent)
%% ONLY WORKS TILL HERE!
%
%% Problem 3
% Sub in numerical test values
An = 0;%meters
Bn = 0.2794;%meters
Cn = 0.3048;%meters
q = deg2rad([0;15;-30]);
titleAdd = 'Problem 3 Position';
T_links_p3 = double(subs(T_links,{A,B,C,q1,q2,q3},[An,Bn,Cn,q']));
T_masses_p3 = double(subs(T_masses,{A,B,C,q1,q2,q3},[An,Bn,Cn,q']));
%%
% In the numerical transformation matrix, the fourth column has units
% of meters.
disp('The transformation matrix to the tip is:')
disp(T_links_p3(:,:,4))

%%end
plotarm(q1,q2,q3,q4,q5,q6,An,Bn,Cn);
%plotArm2D(T_links_p3,T_masses_p3,titleAdd);
%%
% The jacobian of the tip mass at this configuration is 
J_mL_p3 = double(subs(J_mL,{A,B,C,q1,q2,q3},[An,Bn,Cn,q']));
disp(J_mL_p3)
%%
% Cartesian velocity is 
dq = deg2rad([30,30,30]');
dx = J_mL_p3*dq;
fprintf(' %06.3f m/s\n',dx(1:3))
fprintf(' %06.3f rad/s\n',dx(4:6))
%% Prob. 4
% Hopefully I interpret this question correctly: _massless links_
% implies that the only weight from gravity is from the load $m_L$.
% Note that units are in N-m. By specifying the necessary reaction
% force at the tip, we get the required joint torques for gravity
% compensation.
w = [0; mL*g;0;0;0;0];
Tau_p4a = J_mL'*w;
disp('Tau_1:')
vpa(Tau_p4a(1),2)
disp('Tau_2:')
vpa(Tau_p4a(2),2)
disp('Tau_3:')
vpa(Tau_p4a(3),2)
%% 
% Using the configuration from problem 3, with units in N-m $([\tau_1,
% \tau_2, \tau_3]^T):$
mL_p4 = 1.5;%kg
g_n = 9.8;%m/s^2 (numerical value of g)
w = [0; mL_p4*g_n;0;0;0;0];
Tau_p4b = J_mL_p3'*w;
disp(Tau_p4b)
%% Problem 5
% Now multiply through each jacobian with the joint velocities,
% $\omega_i = \dot{q}_i$.
v_mA = simplify(J_mA*[dq1 dq2 dq3]');
v_mB = simplify(J_mB*[dq1 dq2 dq3]');
v_mC = simplify(J_mC*[dq1 dq2 dq3]');
v_mL = simplify(J_mL(1:3,1:3)*[dq1 dq2 dq3]');
fprintf('Velocity of Mass 1:\n\r')
pretty(v_mA);
fprintf('Velocity of Mass 2:\n\r')
pretty(v_mB);
fprintf('Velocity of Mass 3:\n\r')
pretty(v_mC);
fprintf('Velocity of Mass L:\n\r')
pretty(v_mL);
%%
%%% _Kinetic Energy Potential Energy_
% Solve for Kinetic Energy and Potential Energy of the system.
vASqrd = simplify(v_mA'*v_mA);
vBSqrd = simplify(v_mB'*v_mB);
vCSqrd = simplify(v_mC'*v_mC);
vLSqrd = simplify(v_mL'*v_mL);


KEA = (mA*vASqrd)/2;
PEA = mA*g*p_mA(2);%only base frame 'y' component is needed for PE
KEB = (mB*vBSqrd)/2;
PEB = mB*g*p_mB(2);
KEC = (mC*vCSqrd)/2;
PEC = mC*g*p_mC(2);
KEL = (mL*vLSqrd)/2;
PEL = mL*g*p_mL(2);
fprintf('KE of Mass 1:\n\r')
pretty(KEA);
fprintf('PE of Mass 1:\n\r')
pretty(PEA);
fprintf('KE of Mass 2:\n\r')
pretty(KEB);
fprintf('PE of Mass 2:\n\r')
pretty(PEB);
fprintf('KE of Mass 3:\n\r')
pretty(KEC);
fprintf('PE of Mass 3:\n\r')
pretty(PEC);
fprintf('KE of Mass L:\n\r')
pretty(KEL);
fprintf('PE of Mass L:\n\r')
pretty(PEL);

%%
% _The Lagrangian_ using the parameters above is calculated below
LA = KEA - PEA;
LB = KEB - PEB;
LC = KEC - PEC;
LL = KELend - PEL;
L = simplify(LA + LB + LC + LL);
L_p5 =simplify(subs(L,{A,B,C,mA,mB,mC,mL},[0.8,0.4,0.2,2,1,0.5,1.5]));
fprintf('Lagrangian:\n\r');
pretty(L_p5);
%%
%%% Derive the Dynamics from Lagrangian 
% The components of the system
% dynamics orriginate from the equation below where $i \in {1,2,3}$:
% 
% $$\tau_i = \frac{d}{dt}\frac{\partial L_i}{\partial \omega_i} -
% \frac{\partial L_i}{\theta_i}$$
% 
% The dynamics equations require symbolic functions of time. First
% lets create them and then sub them in.
syms th1(t) th2(t) th3(t);%joint positions
syms w1(t) w2(t) w3(t);%joint velocities
L = subs(L_p5,[q1 q2 q3 dq1 dq2 dq3],[th1 th2 th3 w1 w2 w3]);

%%
% Dynamics for w.r.t. joint 1. 
delLDelw1 = functionalDerivative(L,w1);
delLDelTh1 = functionalDerivative(L,th1);
Tau1 = diff(delLDelw1,t) - delLDelTh1;
% Terms to replace
subTauOld = [diff(w1(t), t), diff(w2(t), t), diff(w3(t), t),...
    diff(th1(t), t), diff(th2(t), t), diff(th3(t), t),...
    w1(t), w2(t), w3(t), th1(t), th2(t),th3(t)];
% simpler terms 
subTauNew = [ddq1, ddq2, ddq3, dq1, dq2, dq3, dq1, dq2, dq3,q1,q2,q3];
Tau1 = simplify(subs(Tau1,subTauOld,subTauNew));
Tau1 = collect(Tau1,[q1,q2,q3,dq1,dq2,dq3,ddq1,ddq2,ddq3]);
fprintf('Tau_1:\n\r');
pretty(Tau1);
fprintf('\n\r');
%%
% Dynamics w.r.t. joint 2.
delLDelw2 = functionalDerivative(L,w2);
delLDelTh2 = functionalDerivative(L,th2);
Tau2 = diff(delLDelw2,t) - delLDelTh2;
Tau2 = simplify(subs(Tau2,subTauOld,subTauNew));
Tau2 = collect(Tau2,[q1,q2,q3,dq1,dq2,dq3,ddq1,ddq2,ddq3]);
fprintf('Tau_2:\n\r');
pretty(Tau2);
fprintf('\n\r');
%%
% Dynamics w.r.t. joint 3.
delLDelw3 = functionalDerivative(L,w3);
delLDelTh3 = functionalDerivative(L,th3);
Tau3 = diff(delLDelw3,t) - delLDelTh3;
Tau3 = simplify(subs(Tau3,subTauOld,subTauNew));
Tau3 = collect(Tau3,[q1,q2,q3,dq1,dq2,dq3,ddq1,ddq2,ddq3]);
fprintf('Tau_3:\n\r');
pretty(Tau3);
fprintf('\n\r');

%%
% _Express Dynamics in Generalized form_
% The generalized dynamics of the system are expressed in the form
% of Robot Modelling and Control, EQ 6.61 where D = M:
%
% $$ \vec{\tau} = M(\vec{\theta})\vec{\alpha} +
% C(\vec{\theta},\vec{\omega})\vec{\omega} + g(\vec{\theta})$$
%
% The Inertia Matrix has 9 elements multiplied by joint accelerations
% so we subtract off all other terms from Tau to factor out the
% accelerations. 
M_11 = simplify(Tau1 - subs(Tau1,ddq1,0))/ddq1;
M_12 = simplify(Tau1 - subs(Tau1,ddq2,0))/ddq2;
M_13 = simplify(Tau1 - subs(Tau1,ddq3,0))/ddq3;
M_21 = simplify(Tau2 - subs(Tau2,ddq1,0))/ddq1;
M_22 = simplify(Tau2 - subs(Tau2,ddq2,0))/ddq2;
M_23 = simplify(Tau2 - subs(Tau2,ddq3,0))/ddq3;
M_31 = simplify(Tau3 - subs(Tau3,ddq1,0))/ddq1;
M_32 = simplify(Tau3 - subs(Tau3,ddq2,0))/ddq2;
M_33 = simplify(Tau3 - subs(Tau3,ddq3,0))/ddq3;

M = [M_11, M_12, M_13;...
     M_21, M_22, M_23;...
     M_31, M_32, M_33];
fprintf('M:\n\r');
pretty(M);
fprintf('\n\r');
%%
% And the gravitational vector takes the form:

G = [subs(Tau1,[dq1,dq2,dq3,ddq1,ddq2,ddq3],[0,0,0,0,0,0]);...
     subs(Tau2,[dq1,dq2,dq3,ddq1,ddq2,ddq3],[0,0,0,0,0,0]);...
     subs(Tau3,[dq1,dq2,dq3,ddq1,ddq2,ddq3],[0,0,0,0,0,0])];
     
fprintf('G:\n\r');
disp(G);
fprintf('\n\r');

%%
% The Coriolis Matrix Takes the form of the remaining terms from the
% Torque vector. Note that the vector, $\vec{\dot{q}}$ has been
% multiplied into the C.
TauVect = [Tau1;...
           Tau2;...
           Tau3];
% C*omega = C*dq
Comega = simplify(TauVect - (M*[ddq1,ddq2,ddq3]' + G));
fprintf('C*[qd1;qd2;qd3]:\n\r');
pretty(Comega);
fprintf('\n\r');
%%
% Given the state vector: $\vec{q} = [\theta_1, \theta_2, \theta_3, \omega_1,
% \omega_2,\omega_3]^T$ its time-deriviative gives the equations of motion in
% its bottom two rows.

%% Helper Functions
%%% _dh2mat_
function T = dh2mat(theta,d,a,alpha)
% Create a transformation matrix using DH parameters.
% T = DH2MAT(THETA,D,A,ALPHA) creates a transformation matrix, using
% the Denativ-Hartenberg convention, from frame n-1 to to frame n.
% THETA is the joints axis of revolution, the z-axis in frame n-1. D 
% is the transformation along the z-axis in frame n-1 and A is the
% transformation along the current x-axis to align the the origin of
% the current frame with frame n. ALPHA is the rotation about the
% current x-axis to align the z-axis with the axis of rotation for
% frame n.
%
% INPUT:
%   THETA   - [nx1] joint positions & necessary offset about Z_n-1
%              (radians)
%   D       - [nx1] translations along Z_n-1
%   A       - [nx1] translations along current X
%   ALPHA   - [nx1] rotations about current X (radians)
%
% OUTPUT:
%   T       - [4x4xn] matrix of intermediate transformations
nTheta = numel(theta);
nD = numel(d);
nA = numel(a);
nAlpha = numel(alpha);
if any(diff([nTheta, nD, nA, nAlpha]))
    msg = 'Input parameter lengths need to be equal';
    error(msg)
else
    n = nTheta;
end

T = repmat(eye(4,4),1,1,n);
% check for symbolic inputs
symTh = isa(theta,'sym');
symD = isa(d,'sym');
symA = isa(a,'sym');
symAl = isa(alpha,'sym');
if any([symTh, symD,symA,symAl])
    T = sym(T);
end

for i = 1:n
    ct = cos(theta(i));
    st = sin(theta(i));
    RzTh = [ct, -st, 0, 0;...
            st,  ct, 0, 0;...
             0,   0, 1, 0;...
             0,   0, 0, 1];
    RzTh = simpX(RzTh);
    TzD = eye(4);
    if symD
        TzD = sym(TzD);
    end
    TzD(3,4) = d(i);
    TxA = eye(4);
    if symA
        TxA = sym(TxA);
    end
    TxA(1,4) = a(i);
    ca = cos(alpha(i));
    sa = sin(alpha(i));
    RxAl = [1,  0,   0,   0;...
            0, ca, -sa,   0;...
            0, sa,  ca,   0;...
            0,  0,   0,   1];
    RxAl = simpX(RxAl);
    T(:,:,i) = simpX(RzTh*TzD*TxA*RxAl);

end 
end
%%% _simpX_
function x = simpX(x)
% called in the case of symbolic DH parameters
    if isa(x,'sym')
        x = simplify(x);
    end
end

%%% _plotArm2D_
function [figHand, linkFig, massFig] =...
    plotArm2D(T_links,T_masses,titleAdd,figHand)
%inputs - last 2 only, sorry
% titleAdd - add this string to the figure title
% figHand - plot arm on this figure if included
numLink = size(T_links,3);
numMass = size(T_masses,3);
x=1;% reference indecies
y=2;
% Init space
links = zeros(2,numLink);
masses = links;
% Calc x-y positions of links and masses w/ fwd kin.
for i = 1:numLink
    p_i = T_links(:,:,i)*[0 0 0 1]';
    links(:,i) = p_i(1:2);
end%for
for i = 1:numMass
    p_i = T_masses(:,:,i)*[0 0 0 1]';
    masses(:,i) = p_i(1:2);
end%for
colorOrd = ['b','g','r','m']';
massStyle = strcat(colorOrd,'.');

if nargin == 4
    figure(figHand)
else
    figHand = figure;
end
hold on
for i = 1:numLink-1
    linkFig = plot(links(x,i:i+1),links(y,i:i+1),colorOrd(i),...
        'LineWidth',3);
end
for i = 1:numMass
    massFig = plot(masses(x,i),masses(y,i),massStyle(i,:),...
        'Markersize',28);
end

%axis([-.700,.700,-.700,.700]);
if nargin >= 3
    titleString = strcat(titleAdd,' Arm Pos (m)');
else
    titleString = 'Arm Pos (m)';
end
title(titleString)
grid on
hold off
end

