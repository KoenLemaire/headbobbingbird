function [J] = headbob_cost (x0,parms)
[Wcoll,~,~] = bird_headbob_optim(x0, parms);
% cost is only energy dissipated during collision because this is the only
% place where we can loose energy to the environment; as the total
% mechanical energy does not change (periodic motion), the net positive
% work done by the neck and push off is equal to the collision losses... 
J=-Wcoll; % [J] 
%J=1; % dummy to check constraint optimization ... 