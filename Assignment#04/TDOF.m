clear all
close all
%% SOLVING SPRING MASS SYSTEM IN 3 DOF 


%% STEP     1: Equation of motion of each body

%%
syms m1 m2 m3 k1 k2 k3 x1 x2 x3 ddx1 ddx2 ddx3

% The symbolic computation of each body of system is calculated in this step.
% Formulation of the equation of motion of each body considering the 
% numerical values of masses, and spring stiffnesses has already been computed
% in file "system_eq".

X    = [x1  ;x2  ;x3];
ddX  = [ddx1;ddx2;ddx3];
M = diag([m1,m2,m3]);
M = diag([m1,m2,m3]);
K = [k1+k2, -k2,0;
     -k2, k2+k3,-k3;
     0, -k3,k3];
 
 eom = M*ddX+K*X==0;


%% STEP     2: Calculating the natural frequencies each body

%%

% In this step, the natural frequencies of all bodies are computed. This
% requires the calculation of mass matrix "M" and stiffness matrix "K".
% Vector "d=eig(K,M)" returns to the values by which natural frequencies 
% wn1, wn2, and wn3 are calculated by taking the square roots of "d".


m1= 2   ;
m2= 1.5 ;
m3= 1   ;
k1=20000;
k2=15000;
k3=10000;

M= diag([m1 m2 m3]);
K=[k1+k2 -k2 0;-k2 k2+k3 -k3;0 -k3 k3];
d=eig(K,M);

wn= sqrt(d);               
wn1=wn(1);                  
wn2=wn(2);                 
wn3=wn(3);


%% STEP     3: Removing spring k1 from the system

%%

% Let k1=0 and K_1 is the updated stiffness matrix. Then, new natural
% frequencies are wn1_1,wn2_1 and wn3_1 of masses m1, m2, and m3. 


K_1=[k2 -k2 0;-k2 k2+k3 -k3;0 -k3 k3];
d_1=eig(K_1,M);
wn_1=sqrt(d_1);
wn1_1=wn_1(1);
wn2_1=wn_1(2);
wn3_1=wn_1(3);


%% STEP     4: Solving the system equations using odeRK4,odeSIE,ode45, and 
%                  ode15s solvers  at different time steps
%%

% Let "X0" initial condition, "h" is the time step and tspan is the total 
% time of simulation.

X0        =  [0.5 0.75 0.75 1.25 0.9 1.6];
x0        =  [0.5 0.75 0.75];
y0        =  [1.25 0.9 1.6];
h         =  1e-4;
divFactor =  4;
tk1       =  0:h:0.25;
tk2       =  0:h/divFactor:0.25;
tspan     =  0.25;
opts = odeset ('AbsTol',1e-9,'RelTol',1e-6);

%% STEP     4.1: Solving the using odeRK4 at different time steps

% Solving the system with odeRK4 at h=1e-4
tic
[t1,x1] = odeRK4(@system_eq,[h tspan] ,X0);
toc

t1 = t1';
x1 = x1';
% Solving the system with odeRK4 at h/divFactor
tic
[t2,x2] = odeRK4(@system_eq,[h/divFactor tspan] ,X0);
toc

t2 = t2';
x2 = x2';
%% STEP     4.2: Solving the using odeSIE at different time steps

% As per function file, odeSIE requires two functions fun_g and fun_f.
% Therefore:

fun_g = @(t,y) y; 
fun_f=@(t,x) M\(-K*x);

% Solving the system with odeSIE at h=1e-4
tic
[t3,x3,y3]=odeSIE(fun_f,fun_g,[h tspan] ,x0,y0);
toc

t3 = t3';
x3 = x3';

% Solving the system with odeSIE at h/divFactor
tic
[t4,x4,y4]=odeSIE(fun_f,fun_g,[h/divFactor tspan] ,x0,y0);
toc

t4 = t4';
x4 = x4';
%% STEP     4.3: Solving the using ode45 at different time steps

% Solving the system with ode45 at h=1e-4
tic
[t5,x5]= ode45(@system_eq,tk1,X0,opts);
toc



% Solving the system with ode45 at h/divFactor
tic
[t6,x6]= ode45(@system_eq,tk2,X0,opts);
toc



%% STEP     4.4: Solving the using ode15s at different time steps


% Solving the system with ode15s at h=1e-4
tic
[t7,x7]= ode15s(@system_eq,tk1,X0,opts);
toc



% Solving the system with ode15s at h/divFactor
tic
[t8,x8]= ode15s(@system_eq,tk2,X0,opts);
toc



%% STEP     4.5: Solvers efficiency and the accuracy


compareResults (t1,x1,t2, x2)
compareResults (t3,x3,t4, x4)
compareResults (t5,x5,t6, x6)
compareResults (t7,x7,t8, x8)

%% STEP     5: Computations using modal coordinates p
%% 

%   In this step, the system modal will be computed using modal
%   coordinates. 

[V, L] = eig (K,M);

% OUTPUTS

% V    Eigenvectors
% L    Eigenvalues

% Initial conditions p0 will be calculated from V, x0 and y0 seprately for
% position and velocity.


p00 = V \ x0';
p01 = V \ y0';
p0 = [-1.4529 0.1956 -0.0672 -2.4477 0.1787 -0.0734];        

%%  STEP    5.1: Caclculation of M_p and K_p
M_p= V'*(M*V);
K_p= V'*(K*V);

%%  STEP    5.2: Verification of M_p = I  and K_p = L
% It can be seen from the workspace that the M_p = I and also K_p = L. So
% it is verified. 

%%  STEP    5.2: Comparing of results with original system

% So first step here is to solve the sytem based upon the modal
% coordinates. Then, the results will be compared with original system.

fun_g1=@(t,y) y;
fun_f1= @(t,p) -1*(M_p^-1)*K_p*p;

%% STEP    5.2.1: Solving the system using odeRK4  at different time steps

% Solving the system with odeRK4 at h=1e-4
tic
[t9,x9] = odeRK4(@modal_coordinates,[h tspan] ,p0);
toc

t9=t9';
x9=x9';
% Solving the system with odeRK4 at h/divFactor
tic
[t10,x10] = odeRK4(@modal_coordinates,[h/divFactor tspan] ,p0);
toc

t10=t10';
x10=x10';
%% STEP    5.2.2: Solving the system using odeSIE  at different time steps

% Solving the system with odeSIE at h=1e-4
tic
[t11,x11,y11]=odeSIE(fun_f1,fun_g1,[h tspan] ,p00,p01);
toc

t11=t11';
x11=x11';

% Solving the system with odeSIE at h/divFactor
tic
[t12,x12,y12]=odeSIE(fun_f1,fun_g1,[h/divFactor tspan] ,p00,p01);
toc

t12=t12';
x12=x12';
%% STEP    5.2.3: Solving the using ode45 at different time steps

% Solving the system with ode45 at h=1e-4
tic
[t13,x13]= ode45(@modal_coordinates,tk1,p0,opts);
toc

% Solving the system with ode45 at h/divFactor
tic
[t14,x14]= ode45(@modal_coordinates,tk2,p0,opts);
toc

%% STEP    5.2.4: Solving the using ode15s at different time steps

% Solving the system with ode15s at h=1e-4
tic
[t15,x15]= ode15s(@modal_coordinates,tk1,p0,opts);
toc

% Solving the system with ode15s at h/divFactor
tic
[t16,x16]= ode15s(@modal_coordinates,tk2,p0,opts);
toc

%%   STEP    5.2.5: Comparing the results with original system
% 
compareResults (t1,x1,t9, x9)
compareResults (t3,x3,t11, x11)
compareResults (t5,x5,t13, x13)
compareResults (t7,x7,t15, x15)


%% Plotting the results
%%

figure (1)

plot (t1,x1(:,1),'k',t3,x3(:,1),':k',t5,x5(:,1),'--k',t7,x7(:,1),'-.k','LineWidth',1.5)
hold on


ylabel({'Position (m)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
xlabel({'Time (s)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
title('System solution in normal coordinates','FontUnits','points','FontWeight','normal','FontSize',12,'FontName','Times New Roman','LineWidth',2)



figure (2)
plot (t9,x9(:,1),'k',t11,x11(:,1),':k',t13,x13(:,1),'--k',t15,x15(:,1),'-.k','LineWidth',1.5)


ylabel({'Position (m)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
xlabel({'Time (s)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
title('System solution in modal coordinates','FontUnits','points','FontWeight','normal','FontSize',12,'FontName','Times New Roman','LineWidth',2)




