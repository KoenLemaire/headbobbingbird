% this is a script to test the function newt_root.m
% Koen Lemaire 01/2019

% create anonimous function for which to find roots
% some random parameters
a=2; b=3; c=4; d=5;
% a test function with 3 inputs and 2 outputs
testfun = @(x) [a*x(1)^2 + -b*x(2) + x(3)^5;
                c*x(1)^3 + -x(2)^4 + d*x(3)];

% random initial x vector, note that different roots are found depending on
% initial guess
x=randn(3,1); 

% test testfun:
y=feval(testfun,x)

% find roots:
root=newt_root(testfun,x)

% evaluate if root is really a root:
check=feval(testfun,root)
   