clear all
close all
%% Solving spring mass system in three degree of freedom
%% Writing the equation of motion of system
syms m1 m2 m3 k1 k2 k3 x1 x2 x3 ddx1 ddx2 ddx3

M=[m1 0 0;0 m2 0;0 0 m3];                   % mass matrix
K=[k1+k2 -k2 0;-k2 k2+k3 -k3;0 -k3 k3];     % stiffness matrix
ddx=[ddx1;ddx2;ddx3];                       % acceleration matrix
x=[x1;x2;x3];                               % displacement matrix

Eq = M*ddx+K*x==0;                          % equation of the system
%% System parameters
%%

m1= 2   ;
m2= 1.5 ;
m3= 1   ;
k1=20000;
k2=15000;
k3=10000;

M=[m1 0 0;0 m2 0;0 0 m3];                       % mass matrix for defined variables
K=[k1+k2 -k2 0;-k2 k2+k3 -k3;0 -k3 k3];         % stiffness matrix defined variables
Eq = M*ddx-K*x;
%% Calculating the natural frequencies
%%

d=eig(K,M);
wn= sqrt(d);                %Natural frequency 
wn1=wn(1);                  %Natural frequency of mass m1
wn2=wn(2);                  %Natural frequency of mass m2
wn3=wn(3);                  %Natural frequency of mass m3
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

tspan=0:1e-4:2;
dt=1e-4;                            %time interval
tk=2;                               %total time
x0=[0.5 0.75 0.75 1.25 0.9 1.6];    %initial conditions
                 
fun_g = @(t,y) y;                   %fun_g needed to compute the solution using odeSIE
fun_f=@(t,x) (M^-1)*K*x;            %fun_f needed to compute the solution using odeSIE


%odeRK4
tic
[t1,x1] = odeRK4(@system_eq,[dt tk] ,x0);
toc

t1=t1';
x1=x1';

%odeSIE
tic
[t2,x2,y2]=odeSIE(fun_g,fun_f,[dt tk] ,[0.5 0.75 0.9],[0.75 1.25 1.6]);
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

%% Solvers efficiency and the accuracy
%%  

rmse_RK45= sqrt(sum(x1(:,1)-x3(:,1)).^2/length(x1(:,1)));        % Root mean sqaure error of odeRK4 vs ode45
rmse_RK15= sqrt(sum(x1(:,1)-x4(:,1)).^2/length(x1(:,1)));        % Root mean sqaure error of odeRK4 vs ode15s

rmse_SIE45= sqrt(sum(x2(:,1)-x3(:,1)).^2/length(x2(:,1)));       % Root mean sqaure error of odeSIE vs ode45
rmse_SIE15= sqrt(sum(x2(:,1)-x4(:,1)).^2/length(x2(:,1)));       % Root mean sqaure error of odeSIE vs ode15s


%% Computing using modal coordinates
%% 

K_expand=[0 1 0 0 0 0;k1+k2 0 -k2 0 0 0;0 0 0 1 0 0;-k2 0 k2+k3 0 -k3 0;0 0 0 0 0 1;0 0 -k3 0 k3 0];
M_expand=[0 0 0 0 0 0;0 m1 0 0 0 0;0 0 0 0 0 0;0 0 0 m2 0 0;0 0 0 0 0 0;0 0 0 0 0 m3];
[V,L]=eig(K_expand,M_expand);   %V corresponds to the eigenvetors and diagonal of L corresponds to the eigenvalues
omega=sqrt(V(1,1));
X= V(:,1);
p0=inv(V)*x0';           %modal coordinates p

M_dash= V'*(M_expand*V);
K_dash= V'*(K_expand*V);

fun_g1=@(t,y) y;
fun_f1= @(t,p) -1*(M_dash^-1)*K_dash*p;

%%   Solving the equation of modal coordinates
%%

%odeRK4
[t5,x5] = odeRK4(@modal_coordinates,[dt tk] ,p0);
t5=t5';
x5=x5';

%odeSIE
[t6,x6,y6]=odeSIE(fun_g,fun_f,[dt tk] ,[-2.85e15 -2.54e15 2.85e15],[-3.23e15 3.23315 2.54e15]);
t6=t6';
x6=x6';

%ode45
[t7,x7]= ode45(@modal_coordinates,tspan,p0);


%ode15s
[t8,x8]= ode15s(@modal_coordinates,tspan,p0);

%% Plotting the results
%%

figure (1)

plot (t3,x3(:,1))
hold on


ylabel({'Position (m)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
xlabel({'Time (s)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
title('System solution','FontUnits','points','FontWeight','normal','FontSize',12,'FontName','Times New Roman','LineWidth',2)



figure (2)
plot (t7,x7(:,1))


ylabel({'Position (m)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
xlabel({'Time (s)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
title('Model coorinate solution','FontUnits','points','FontWeight','normal','FontSize',12,'FontName','Times New Roman','LineWidth',2)




