function [C,Ceq] = headbob_nonlincon (x0,parms)
[~,c_coll,state_end] = bird_headbob_optim(x0, parms);
% inequality constrain: (scalar) collision impulse in stance leg direction,
% we want this number to be positive. 
C=-c_coll; %[Ns]
%C=[];
% equality constraint on initial and final state
Ceq=state_end(1:2)'-[0.5*pi-parms.alpha; x0(1)];
% potentially add simulation time as a constraint (to enforce speed??)
% Ceq(3,1)=x0(3)-parms.step_time; % perhaps??
    


