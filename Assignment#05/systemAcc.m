function [xpp] = systemAcc (x, xp, m, k, c, L, g)

%SYSTEM FUNCTION COMPUTES SYSTEM ACCELERATION

% INPUTS 
% x         - Position
% xp        - Velocity
% m         - Mass
% k         - Spring coefficient
% c         - Damping coefficient
% L         - Length
% g         - Gravity accelration


M = [3*m, m*L; m*L, 2*m*L*L/3];
C = [2*c, c*L; c*L, c*L*L];
K = [3*k, 2*k*L; 2*k*L, 2*k*L*L + m*g*L];
xpp = -M\(C*xp + K*x);
end
