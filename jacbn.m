function [J] = jacbn(fun,x,varargin)
% [J] = jacbn(fun,x,pert_size)
% 
% jacbn evaluates the jacobian [J] of function fun at input x, using
% central differences. fun is a function handle which accepts (column)
% vector x and returns (column) vector y such that y=fun(x). pert_size is
% the perturbation size used to evaluate the numerical derivatives.
% pert_size is optional, default value is 1e-6. Note that passing extra
% parameters to fun has to be done using anonimous functions. 

% Koen Lemaire 01/2019 (KKL)

% first get input and output dimentsions, and test fun:
y=feval(fun,x); % function test
n_outputs=length(y); % nr outputs
n_inputs=length(x); % nr inputs

% decide on perturbation size
if nargin>2 % user supplied perturbation size
    pert_size=varargin{1};
else
    pert_size=1e-6; % default value (based on test function, play around)
end
    
% now we are going to fill each column of J by perturbing each entry of x
J=nan(n_outputs,n_inputs); % initialize J

for i=1:n_inputs % loop over inputs
    % create delta x
    delta_x=zeros(size(x)); % initialize 
    delta_x(i)=pert_size; % perturb ith input variable
    
    % compute delta y using central difference 
    delta_y = feval(fun,x+delta_x) - feval(fun,x-delta_x);
    
    % now compute one column of J. division by 2 delta_x because of central
    % difference
    J(:,i)=delta_y/(2*pert_size); 
end