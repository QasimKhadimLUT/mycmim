
c= 3 ;
h = 4;
fun = @(theta) c*tan(theta)-h; 
dfun = @(theta) c*(sec(theta))^2; 
theta0 = pi/4; 
 % NewtonRaphson(fun, dfun, x0) 
[theta, flag] = NewtonRaphson(fun, dfun, theta0) 
