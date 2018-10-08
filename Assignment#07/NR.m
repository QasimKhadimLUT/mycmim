function x = NR   (Fun,dFun,x0,tol,maxIter)

x = x0;
i = 0;

while any(abs(Fun(x))> tol)
    x = x - (dFun(x)\Fun(x));
    i = i+1;
    
    if i>=maxIter
        disp('Maximum number of iterations reached');
        return
       end
end
