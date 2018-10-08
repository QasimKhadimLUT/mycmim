function JFD = JacobianDiff(ConstEq ,x,h)

JFD = zeros (length(x),length (x));

%% Defining the Jacobian for joints
for i = 1: length (x)
    x1 = x;
    x1(i) = x1(i)+h;
    Fun   = ConstEq(x1);
    JFD(:,i) = (Fun-ConstEq(x))/h;
end
