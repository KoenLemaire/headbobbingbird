function root = newt_root(fun,x)

% root = newt_root(fun,x0)
% 
% function newt_root attemps to find a root of function fun using Newton's
% root finding method, starting at initial guess x. fun accepts input x 
% (column vector) and returns output y (column vector) such that y=fun(x)
% and if succesful 0=fun(root). Note that passing extra parameters to fun
% has to be done using anonimous functions. 

% test function and initiate root finding
y=feval(fun,x);

while max(abs(y))>1e-14 % tolerance
    J=jacbn(fun,x); % compute Jacobian at current x
    delta_x=-J\y; % change in x
    x=x+delta_x; % update x to new point
    y=feval(fun,x); % compute function value at new x
end
root=x; % declare output

    
