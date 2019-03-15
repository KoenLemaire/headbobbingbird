% this is a script to test the function jacbn.m
% Koen Lemaire 01/2019

% create anonimous function to be evaluated by jacbn:
% some random parameters
a=2; b=3; c=4; d=5;
% a test function with 3 inputs and 2 outputs
testfun = @(x) [a*x(1)^2 + -b*x(2) + x(3)^5;
                c*x(1)^3 + -x(2)^4 + d*x(3)];

%x=randn(3,1); % random 3x1 vector
x=[1 2 3]'; % simple x vector
pert_size=1e-4; % perturbation size (optional input argument)

% test testfun:
y=feval(testfun,x)

% analytical jacobian of testfun:
ana_J=[2*a*x(1)    -b          5*x(3)^4;
       3*c*x(1)^2  -4*x(2)^3   d]

% numerical jacobian of testfun:
num_J=jacbn(testfun,x,pert_size)

% difference between numerical and analytical J:
diff_numJ_anaJ=num_J-ana_J


   