close all
clear all
%% Derive rotation matrix

syms alpha beta gamma phi psi theta real

Az(alpha)=[cos(alpha) -sin(alpha) 0;sin(alpha) cos(alpha) 0;0 0 1];
Ax(beta)=[1 0 0;0 cos(beta) -sin(beta);0 sin(beta) cos(beta)];
Az(gamma)=[cos(gamma) -sin(gamma) 0;sin(gamma) cos(gamma) 0;0 0 1];

A = Az(alpha)*Ax(beta)*Az(gamma);

alpha = pi/4;
beta = pi/4;
gamma = pi/4;

A = Az(alpha)*Ax(beta)*Az(gamma);

%% Calculate the Euler angles (zxz) and Euler parameters 

theta= pi/2;
phi=pi/6;
psi=-pi/2;

EA=[theta phi psi];

e_0= cos(1/2*(theta+psi))*cos(1/2*phi);
e_1= cos(1/2*(theta-psi))*sin(1/2*phi);
e_2= sin(1/2*(theta-psi))*sin(1/2*phi);
e_3= sin(1/2*(theta+psi))*cos(1/2*phi);

E=[e_0 e_1 e_2 e_3];


%% Assuming that vector q depends upon time, derive all elements of the total derivatives of dC 

syms x y z phi1 phi2 phi3 t dx dy dz dphi1 dphi2 dphi3 


q=[x;y;z;phi1;phi2;phi3];
dq = [dx;dy;dz;dphi1;dphi2;dphi3];
C=[x^2+y+sqrt(z)+sin(phi1);x*y+x*z+y*sin(phi3)+t^3;sin(phi2)+x^3/2+t];
Cq=jacobian(C,q);
Ct = diff(C,t);
dC=Cq*dq+Ct;

%% %% Assuming that vector q depends upon time, derive all elements of the total derivatives of ddC 

syms ddx ddy ddz ddphi1 ddphi2 ddphi3 

Ctt = diff(Ct,t);
ddq = [ddx;ddy;ddz;ddphi1;ddphi2;ddphi3];
D=jacobian(Cq*dq,q);
Cqt=diff(Cq,t);

ddC=Cq*ddq+D*dq+2*Cqt*dq+Ctt;




