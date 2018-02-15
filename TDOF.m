clear all
close all
%% Solving spring mass system in three degree of freedom
%% Writing the equation of motion of system
syms m1 m2 m3 k1 k2 k3 x1 x2 x3 ddx1 ddx2 ddx3

M=[m1 0 0;0 m2 0;0 0 m3];
K=[k1+k2 -k2 0;-k2 k2+k3 -k3;0 -k3 k3];
ddx=[ddx1;ddx2;ddx3];
x=[x1;x2;x3];

Eq = M*ddx+K*x==0;
%% System parameters
%%

m1= 2   ;
m2= 1.5 ;
m3= 1   ;
k1=20000;
k2=15000;
k3=10000;

M=[m1 0 0;0 m2 0;0 0 m3];
K=[k1+k2 -k2 0;-k2 k2+k3 -k3;0 -k3 k3];
Eq = M*ddx-K*x;
%% Calculating the natural frequencies
%%

d=eig(K,M);
wn= sqrt(d);
wn1=wn(1);
wn2=wn(2);
wn3=wn(3);
%% Removing sprin k1 from the system
%%

K_1=[k2 -k2 0;-k2 k2+k3 -k3;0 -k3 k3];

d_1=eig(K_1,M);
wn_1=sqrt(d_1);
wn1_1=wn_1(1);
wn2_1=wn_1(2);
wn3_1=wn_1(3);

%% Solving the system equations using odeRK4,odeSIE,ode45,ode15s solvers
%%

tspan=0:1e-5:0.3;
dt=1e-5;
tk=0.3;
x0=[0.5 0.75 0.75 1.25 0.9 1.6];
y0=[0 0 0 0 0 0];

%odeRK4
tic
[t1,x1] = odeRK4(@system_eq,[dt tk] ,x0);
toc

t1=t1';
x1=x1';

%odeSIE
tic
[t2,x2]=odeSIE(@system_eq,[dt tk] ,x0,y0);
toc

t2=t2';
x2=x2';

%ode45
tic
[t3,x3]= ode45(@system_eq,tspan,x0);
toc


%ode15s
tic
[t4,x4]= ode15s(@system_eq,tspan,x0);
toc




%% Computing using modal coordinates
%% 

[V,L]=eig(K,M);   %V corresponds to the eigenvetors and diagonal of L corresponds to the eigenvalues
omega=sqrt(V(1,1));
X= V(:,1);
% X=V*p;              %modal coordinates X

M_dash= V'*(M*V);
K_dash= V'*(K*V);

%% Plotting the results
%

figure(1)

plot(t1,x1(:,1),'r');
hold on

plot(t2,x2(:,1),'--');
hold on

plot(t3,x3(:,1),'k-.');
hold on

plot(t4,x4(:,1),'g:');
grid on


ylabel({'Position (m)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman')
xlabel({'Time (s)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman')
title('Position vs Time','FontUnits','points','FontWeight','normal','FontSize',12,'FontName','Times New Roman')

hold off

print('Z:\cmim2018\lut_cmim2018','-dmeta')

%% Time step and error
%%

rmse_RK45= sqrt(sum(x1(:,1)-x3(:,1)).^2/length(x1(:,1)))
rmse_RK15= sqrt(sum(x1(:,1)-x4(:,1)).^2/length(x1(:,1)))

rmse_SIE45= sqrt(sum(x2(:,1)-x3(:,1)).^2/length(x2(:,1)))
rmse_SIE15= sqrt(sum(x2(:,1)-x4(:,1)).^2/length(x2(:,1)))







