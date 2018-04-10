function [yp] = sytstemOdeFun (y, param)

%SYSTEMODEFUN FUNCTION INPUT FOR SYSTEM ODE SOLVER
yp = [y(3:4); systemAcc(y(1:2), y(3:4), param.m, param.k, param.c, param.L, param.g )];
end
