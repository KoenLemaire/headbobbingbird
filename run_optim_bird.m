clear
clc
close all
%% Set parameters
parms.g=-9.81; % [m/s^2] gravitational acceleration
pigeon_mass=1; % [kg] total pigeon mass 
head_mass_rel=.1; % [] proportion of head mass relative to total pigeon mass
hh_rel=1.05; % head height relative to leg length
bobtime_rel=.25; % [] relative time duration of headmotion  
parms.L=1; % [m] leg length
parms.alpha=.3; % [rad] half leg spread angle
parms.xh0=0.2*parms.L;
%% (potential) sweeping parameters:
delayVec=linspace(0.1,0.6,10);
for iDelay=4;%1:length(delayVec)
delay_rel=delayVec(iDelay);
parms.speed=2*parms.L; % [m/s] desired walking speed

%% derived parameters
parms.mp=(1-head_mass_rel)*pigeon_mass; % [kg] pigeon pelvis mass   
parms.mh=head_mass_rel*pigeon_mass; % [kg] pigeon head mass
parms.hh=hh_rel*parms.L; % [m] head height 
parms.step_length=2*parms.L*sin(parms.alpha); % [m] step length
parms.step_time=parms.step_length/parms.speed; % [s] desired step duration
parms.bobtime=parms.step_time*bobtime_rel; % [s]
parms.delay=delay_rel*parms.step_time; % [s]
%% free parameters
% potentially add simulation time ??
phid0=-2.5; % [rad/s] phidot of stance leg AT END OF PREVIOUS STANCE PHASE
Phat_push=0.5; % [Ns] push off magnitude
step_time=parms.step_length/parms.speed;
%x0 = [phid0; Phat_push; step_time]; % initial guess for design parms
x0 = [phid0; Phat_push]; % initial guess for design parms
%% test cost function
%[Wcoll,c_coll,state_end,t,state,Ekin_stance,Wgravity,Wneck,Wpush] = bird_headbob_optim(x0, parms);
%[t,state,phi_minus,phi_plus,phid_minus,phid_plus,Ekin_stance,Wgravity,Wneck,Wcoll,Wpush,Ekin_plus] = bird_headbob(x0, parms)
%% run optimization
if 1
%fmincon_opt.FiniteDifferenceStepSize=1e-7;
%fmincon_opt.StepTolerance=1e-6;
%fmincon_opt.FiniteDifferenceType='central';
fmincon_opt.Algorithm='active-set';
%fmincon_opt.Algorithm='interior-point';
fmincon_opt.Display='iter';

optimFun=@(x)headbob_cost(x,parms); % DOESNT WORK WITH LSQNONLIN!!!
nonlinconFun=@(x)headbob_nonlincon(x,parms);
[x,fval(iDelay),flag] = fmincon(optimFun,x0,[],[],[],[],[],[],nonlinconFun,fmincon_opt);

[C,Ceq] = headbob_nonlincon (x,parms);
%[J] = headbob_cost (x,parms)
[Wcoll,c_coll,state_end,t,state,Ekin_stance,Wgravity,Wneck,Wpush] = bird_headbob_optim(x, parms);
end
end
%% State plots; not really a test, but demonstration plot
phi=state(:,1);
phid=state(:,2);
xh=state(:,3);
yh=parms.hh;

figure;
subplot(121);plot(t,state(:,1:2),'linewidth',2)
xlabel('Time [s]');
ylabel('leg angles [rad, rad/s]');
title('states of leg vs time of stance phase for headbobbing bird');
legend('phi','phidot')
subplot(122);plot(t,state(:,3:4),'linewidth',2)
xlabel('Time [s]');
ylabel('headbob x coordinate [m, m/s]');
title('states of head vs time of stance phase for headbobbing bird');
legend('xh','xhdot')

figure;
subplot(121);plot(state(:,1),state(:,2),'linewidth',2)
xlabel('phi [rad]');
ylabel('phidot [rad/s]');
title('phase diagram of leg during stance phase');
subplot(122);plot(state(:,3),state(:,4),'linewidth',2)
xlabel('xh [m]');
ylabel('xhdot [m/s]');
title('phase diagram of head during stance phase');
%% Energy plot
figure
subplot(121);plot(t(2:end),Ekin_stance-Wgravity-Wneck,'linewidth',2)
xlabel('Time [s]');
ylabel('delta Ekin - Wtot [J]');
title('Energy balance of stance phase for headbobbing bird');
subplot(122);plot(t(2:end),Wneck,'linewidth',2)
xlabel('Time [s]');
ylabel('Wneck [J]');
title('Active work done by neck'); 

%% animation
if 1
figure;
for iStep=1:length(t)
    r_p=parms.L*[cos(phi(iStep)) sin(phi(iStep))];
    plot([0 r_p(1)],[0 r_p(2)],'k'); hold on
    plot(xh(iStep),yh,'ro','linewidth',2); hold on
    plot(r_p(1),r_p(2),'ko','linewidth',2,'markersize',8); hold on
    plot([r_p(1) xh(iStep)],[r_p(2) yh],'b--'); hold on
    xlabel('x axis [m]')
    ylabel('y axis [m]')
    legend('leg','head','pelvis','neck')
    title('stick diagram of stance leg and head')
    axis([-1.2 1.2 -1.2 1.2]*parms.L)
    drawnow
    axis equal
end
end