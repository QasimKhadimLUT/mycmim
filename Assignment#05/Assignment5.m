close all
clear all

%% System equations
%%

syms x(t) theta(t) g  m c k L 
assumeAlso (m >0)
assumeAlso (c >0)
assumeAlso (k >0)

xp=diff(x,t);
xpp=diff(xp,t);

thetap=diff(theta,t);
thetapp=diff(thetap,t);

Eq_1=3*m*xpp+2*c*xp+c*L*thetap+3*k*x+2*k*L*theta;
Eq_2=((m*L^2)/3)*thetapp+c*L*xp+c*(L^2)*thetap+2*k*L*x+2*k*L^2*theta-2*m*g;

M=[3*m 0;0 (m*L^2)/3];
C=[2*c c*L;c*L c*L^2];
K=[3*k 2*k*L;2*k*L 2*k*L^2];
F=[0 2*m*g]';

%%  Derive Jacobean matrix
%%
% syms x(t) theta(t)
% 
% J=jacobian([Eq_1,Eq_2], [x,theta])
J(1,1)=functionalDerivative(Eq_1,[x, theta]')
% J(1,2)=functionalDerivative(Eq_1,theta)

% diff(Eq_1,x)




% [t,y]=beul(@volt,@dvolt,[2,1],0,10,1000);
% 
% plot(t,y)