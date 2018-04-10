function [t,y]= odeBE(fun, tspan,y0,dfun)

% IMPLEMENTATION OF THE BACKWARD EULER METHOD

% INPUTS 

% t            - output time points
% y            - output solution values
% fun          - function to integrate
% tspan(dt, tk)- time step and final time

% OUTPUTS
% y0           - initial condition   
% dfun         - Jacobean of fun


% defining time step and final time
dt = tspan(1);
tk = tspan(2);
t =  (0:dt:tk)';

% defining the size of output-y matrix
n = length(t);
m = length(y0);
y = zeros (n,m);

% Implementation of Backward euler-method using Newton Raphson 
% In this approach, basically F(x)=x-u(n)-h*f(x(h+1,x) and 
% JF= I -h*D*f(x) are comouted. Therefore: 

I = eye(m);

% for inital condition

y(1,:) = y0(:)';
yi     = y0(:);

for i = 2:n
    gf = @(x) x-yi-dt*fun(t(i),x);
    dgf= @(x) I-dt*dfun(t(i),x);
    
    % Backward euler method using Newton Raphson method
    [yi,iflag] = NewtonRaphson (gf, dgf, yi+dt*fun(t(i-1),yi));
    
    % To describe if solution is obtained or not. 
    if iflag
        y(i,:)=yi';
    else
        t= t(1:i-1);
        y= y(1:i-1, :);
        break;
    end
end

end
