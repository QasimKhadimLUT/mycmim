clear all
close all
%% DEFINING SYSTEM PARAMETERS

syms m k c L g real

% System posutions (D.O.F) and velocities 

x   =   sym('x', [2, 1], 'real');
xp  =   sym('xp', [2, 1], 'real');

y = [x; xp];

param = struct('m', m, 'k', k, 'c', c, 'L', L, 'g', g);

Fun = systemOdeFun(y, param);

%% COMPUTING JACOBIAN MATRIX AND CREATING MATLAB FUNCTION

FunDy = jacobian(Fun,y);

matlabFunction(FunDy, 'File', 'systemOdeJac', 'Vars', {struct2array(param)});