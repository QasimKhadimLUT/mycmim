
c= 3 ;
h = 4;
fun = @(r) c^2 + h^2 - r^2; 
dfun = @(r) -2*r; 
r0 = 4; 
 % NewtonRaphson(fun, dfun, x0) 
[r, flag] = NewtonRaphson(fun, dfun, r0) 
