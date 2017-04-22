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