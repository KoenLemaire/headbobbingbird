% We are going to do a simulation of the stance phase of a pigeon walking
% with a head on top. For now the head is stationary and we just do
% simulation of the stance phase. 
clear 
clc
close all

% independent parameters
parms.g=-9.81; % [m/s^2] gravitational acceleration
pigeon_mass=1; % [kg] pigeon mass 
head_mass_rel=.2; % [] proportion of head mass relative to total pigeon mass
hh_rel=1.05;
parms.L=1; % [m] leg length
parms.alpha=.3; % [rad] half leg spread angle
parms.speed=2; % [m/s] desired walking speed

% derived parameters
parms.mp=(1-head_mass_rel)*pigeon_mass; % [kg] pigeon pelvis mass   
parms.mh=head_mass_rel*pigeon_mass; % [kg] pigeon head mass
parms.hh=hh_rel*parms.L; % [m] head height 
parms.step_length=2*parms.L*sin(parms.alpha); % [m] step length
parms.step_time=parms.step_length/parms.speed; % [s] desired step duration

% initial condition (maybe make this a parameter input?? to distinguish
% from initial guess in an optimation scheme ...??)
x0 = [.5*pi+parms.alpha; -1; 1*parms.L; .1]; % [phi phid xh xhd] a sample initial state
[t,state,Ekin,Wtot] = bird_stance(x0, parms);

phi=state(:,1);
phid=state(:,2);
xh=state(:,3);
yh=parms.hh;

%% State plot; not really a test, but demonstration plot
figure;
plot(t,state(:,1:2))

figure;
for iStep=1:length(t)
    r_p=parms.L*[cos(phi(iStep)) sin(phi(iStep))];
    plot([0 r_p(1)],[0 r_p(2)],'k',xh(iStep),yh,'ro',[r_p(1) xh(iStep)],[r_p(2) yh],'b--'); hold on
    drawnow
    axis equal
end
% xlabel('time (dimensionless)');
% ylabel('states (dimensionless)');
% title('states vs. time for rimless wheel');
% legend('theta', 'thetadot');

%% Energy plot
figure
plot(t,Ekin-Wtot)
% xlabel('time (dimensionless)');
% ylabel('energy (dimensionless)');
% title('energies vs. time for rimless wheel');
% 
% subplot(122)
% plot(ts,energies) % zoom in on energy for the last step
% xlabel('time (dimensionless)');
% ylabel('energy (dimensionless)');
% title('energies vs. time for final step of rimless wheel');

%% Thetadot per step 
% figure
% plot(0:length(x0s)-1, x0s(:,2),'.')
% xlabel('step number');
% ylabel('initial thetadot (dimensionless)');
% title('initial thetadot vs. step number');
% hold on
% plot([0,length(x0s)-1], thetadotplusstar*[1 1],':')
% legend('thetadot', 'analytical limit cycle')


