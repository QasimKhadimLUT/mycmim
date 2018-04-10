function [ xout, flag ] = NewtonRaphson( fun, dfun, x0, maxiter, tol)

% SIMPLE IMPLEMENTATION OF NR METHOD

% INPUTS
% fun           - vector function f(x)
% dfun          - df/dx (Jacobian matrix)
% x0            - initial point
% maxiter       - optimal maximum number of iterations (extra parameter)
% tol           - optional tolerances (extra parameter)

% OUTPUT 
% xout          - converged values
% flag          - indicates if computations are successfull or not

% number of input arguments
if nargin < 4
    maxiter = 30;
end

% if tolerances are not given 
if nargin < 5
    tol = eps(max(abs(x0)))^0.8;
end

% initial values
iter    = 0;
xc      = x0;
fval    = fun(xc);
solnorm = norm(fval);


while solnorm > tol && iter < maxiter
    dfval     = dfun (xc);
    xc        = xc- dfval\fval;
    fval      = fun(xc);
    solnorm  = norm(fval);
    iter = iter + 1;
end

% if solution looks correct, assign solution to output
if solnorm <= tol && ~isnan(solnorm)
    xout = xc;
    flag = true;
else 
    flag = false;
    xout = x0;
end


end
