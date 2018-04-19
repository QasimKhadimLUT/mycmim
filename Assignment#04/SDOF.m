clear all
close all
%%  SPRING MASS SYSTEM


%  The equation of motion of spring mass system is written in SpringMass
%  function file. Now the system definition containing mass "m", stiffness 
%  "k"  and variables explaining the time span such as h,divFactor, tk1,
%  tk2, tspan are expressed.


m = 1;
k=1;
h         =  1e-3;
divFactor =  4;
tk1       =  0:h:5;
tk2       =  0:h/divFactor:5;
tspan     =  5;


% Initial condoition x0 fro odeFE,ode45 and ode15s. x10 and y10 are used
% for odeSIE.

x0=[1;0];
x10=1;
y10=0;

%% STEP     3.1: Solving the using odeFE at different time steps
%%

% Solving the system with odeFE at h=1e-4
tic
[t1,x1] = odeFE(@SpringMass,[h tspan] ,x0);
toc

% Solving the system with odeFE at h/divFactor
tic
[t2,x2] = odeFE(@SpringMass,[h/divFactor tspan] ,x0);
toc

% Solving the system with odeFE at h/(2*divFactor)
tic
[t3,x3] = odeFE(@SpringMass,[h/(2*divFactor) tspan] ,x0);
toc

%% STEP     3.2: Total Energy of Spring Mass systtem
%% Total energy is the sum of kinetic energy (E1) and potential energy(E2).
%  Total energy will be computed for each time step.

E1= 0.5*m* (x1(2,:)).*x1(2,:);
E2 = 0.5*k*(x1(1,:)).*x1(1,:);
E3= E1+E2;

E4= 0.5*m* (x2(2,:)).*x2(2,:);
E5 = 0.5*k*(x2(1,:)).*x2(1,:);
E6= E1+E2;

E7= 0.5*m* (x3(2,:)).*x3(2,:);
E8 = 0.5*k*(x3(1,:)).*x3(1,:);
E9= E1+E2;

%% STEP     4: Solving the using odeSIE at different time steps
%%

% Solving the system with odeSIE at h=1e-4
tic
[t4,x4] = odeSIE(@(t,x) -1*x,@(t,x) x,[h tspan] ,x10,y10);
toc

% Solving the system with odeSIE at h/divFactor
tic
[t5,x5] = odeSIE(@(t,x) -1*x,@(t,x) x,[h/divFactor tspan] ,x10,y10);
toc

% Solving the system with odeSIE at h/(2*divFactor)
tic
[t6,x6] = odeSIE(@(t,x) -1*x,@(t,x) x,[h/(2*divFactor) tspan] ,x10,y10);
toc

%% STEP     5: Solving the using odeRK4 at different time steps
%%

% Solving the system with odeRK4 at h=1e-4
tic
[t7,x7] = odeRK4(@SpringMass,[h tspan] ,x0);
toc

% Solving the system with odeRK4 at h/divFactor
tic
[t8,x8] = odeRK4(@SpringMass,[h/divFactor tspan] ,x0);
toc

% Solving the system with odeRK4 at h/(2*divFactor)
tic
[t9,x9] = odeRK4(@SpringMass,[h/(2*divFactor) tspan] ,x0);
toc

%% STEP     5.2: Comparing odeRK4 and odeSIE
%% 
compareResults (t1,x1,t7, x7)
compareResults (t2,x2,t8, x8)
compareResults (t3,x3,t9, x9)

%% Plotting the solution
%%

figure (1)
plot (t1,x1(1,:),'--k',t1,E3,'-k', t2,x2(1,:),'-.k',t1,E6,'-k',t3,x3(1,:),':k',t1,E9,'-k','LineWidth',2);
ylabel({'Position (m) and Energy (J)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
xlabel({'Time (s)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
title('FE Method','FontUnits','points','FontWeight','normal','FontSize',12,'FontName','Times New Roman','LineWidth',2)
grid on
hold on

figure (2)
plot (t4,x4,'--k',t5,x5,'-.k',t6,x6,':k','LineWidth',2);
ylabel({'Position (m)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
xlabel({'Time (s)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
title('odeSIE Method','FontUnits','points','FontWeight','normal','FontSize',12,'FontName','Times New Roman','LineWidth',2)
grid on
hold on

figure (3)
plot (t7,x7(1,:),'--k',t8,x8(1,:),'-.k',t9,x9(1,:),':k','LineWidth',2);
ylabel({'Position (m)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
xlabel({'Time (s)'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman','LineWidth',2)
title('odeRK4 Method','FontUnits','points','FontWeight','normal','FontSize',12,'FontName','Times New Roman','LineWidth',2)
grid on
hold off
