clear all
close all

%% TESTING NR-METHOD

fun     = @(x) 5.*x^3+x.^2-9;
dfun    = @(x) 15.*x^2+2.*x;
x0      = 1000;

[xr, flag]= NewtonRaphson(fun, dfun,x0);

%% TESTING odeBE

r       = 0.5;
u0      = 2;
ofun    = @(t,x) r*x;
odfun   = @(t,x) r  ;

[t, y] = odeBE(ofun, [1e-5, 1], u0, odfun);

% Exact solution
osol = u0*exp(r*t);

% Plotting the solution
figure (1)
plot(t,y,t,osol,'--','LineWidth',1.5)


%% SECOND ODE TEST

osol2 = @(t) 3/5 - 3/10*exp(-t) .* sin(2.*t) - 3/5.*exp(-t).*cos(2.*t);

ofun2  = @(t,y) [3-2*y(1)-5*y(2); y(1)];
odfun2 = @(t,y) [-2,-5;1,0];

[t2,y2]=odeBE(ofun2, [1e-5, 1], [0;0], odfun2);

osol2val = osol2(t2);
figure (2)
plot(t2, y2(:, 2), t2, osol2val, '--', 'LineWidth', 1.5)

