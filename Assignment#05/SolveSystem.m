close all
clear all

%% SOLVING THE SYSTEM USING BACKWARD EULER METHOD USING NEWTON RAPHSON 

param = struct('m', 2, 'k', 134, 'c', 12, 'L', 0.6, 'g', 9.81);

odeFun = @(~, y) systemOdeFun(y, param);
odeJac = @(~, ~) systemOdeJac(param);

% defining the initial conditions
y0 = [0; 0; 1; 0.02]; 

% defining the timespan

dt = 1e-3;
tk = 1;

disp('odeBE elasped time')

tic
[t1, y1] = odeBE(odeFun,[dt, tk],y0, odeJac);
toc

%% PLOTTING THE RESULTS
figure (1)
plot (t1, y1(:,2),'-.k','LineWidth',1.5)
grid on
hold on

ylabel({'Position'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman')
xlabel({'Time'},'FontUnits','points','interpreter','latex','FontSize',12,'FontName','Times New Roman')
title('Figure 1','FontUnits','points','FontWeight','normal','FontSize',12,'FontName','Times New Roman')

%% CHECKING THE CONVERGENCE AND COMPARING THE RESULTS 

% Computing for shorter time period
divFactor = 4;
[t2, y2] = odeBE(odeFun, [dt/divFactor, tk], y0, odeJac);

% Comparing the results of odeBE 
disp('Compare odeBE for different time step')
compareResults( t1, y1, t2, y2 )


%% SOLVING THE SYSTEM USING ODE15S

disp('ode15s-built in matlab solver')

% Adding optional parameters
opts = odeset ('AbsTol',1e-9,'RelTol',1e-6);

tic
[t15s, y15s]=ode15s (odeFun, t1, y0, opts);
toc

% Comparing ode15s with odeBE
compareResults (t1,y1,t15s, y15s)

%% USING ODE15S WITH JACOBIAN MATRIX


disp('ode15s with Jacobian Matrix')

% Optional parameters
opts = odeset (opts,'Jacobian', odeJac);

% Comparing ode15s with Jacobian
tic
[t15sJ, y15sJ] = ode15s(odeFun, t1, y0, opts);
toc

%% COMPARING RESULTS OF ODEBE AND ODE15S WITH ODE15S WITH JACOBIAN

% comparing odeBE with ode15s with Jacobian
disp('Compare between ode15s-es')
compareResults( t1, y1, t15sJ, y15sJ )

% comparing ode15s with ode15s with Jacobian
disp('Compare between ode15s-es')
compareResults( t15s, y15s, t15sJ, y15sJ )










