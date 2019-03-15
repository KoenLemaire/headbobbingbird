% run consecutive steps for rimless wheel

% Set a tolerance for testing
abstol = 1e-4;

% Use these parameter values
parms = struct('alpha', 0.3, 'rgyr', 0, 'gamma', 0.04, 'tmax', 5);

x0 = [0.3; -0.5]; % a sample initial state

N = 20;
x = [];  % store a big vector of states for multiple steps
t = [];  % big time vector
e = [];  % big energies vector

x0s = x0'; % store the initial states

xnext = x0;
tprev = 0; % cumulative time of previous step end
for i = 1:N
    [xnext, te, xs, ts, energies] = rimlesswheelstep(xnext);
    x = [x; xs]; % states are stored as a column of thetas, col of thetadots
    x0s = [x0s; xnext']; % only the 
    t = [t; ts+tprev];
    e = [e; energies];
    tprev = t(end); % update cumulative time
end

%% Test energy conservation of final step
% We expect energy to be conserved for rimless wheel
assert(std(energies) < abstol, 'Rimless wheel simulation does not conserve energy accurately');

%% Test analytical solution
% whether initial thetadot after N consecutive steps matches
% analytical solution

% Analytical solution predicts thetaplus
thetadotplusstar = -sqrt(4*sin(parms.alpha)*parms.gamma/sin(2*parms.alpha)^2)*cos(2*parms.alpha);
thetadotfinal = x0s(end,2); % thetadotplus for the step following all 
% simulated steps

assert(abs(thetadotplusstar - thetadotfinal) < abstol,'error');

%% State plot; not really a test, but demonstration plot
figure;
plot(t,x)
xlabel('time (dimensionless)');
ylabel('states (dimensionless)');
title('states vs. time for rimless wheel');
legend('theta', 'thetadot');

%% Energy plot
figure
subplot(121)
plot(t,e, '.')
xlabel('time (dimensionless)');
ylabel('energy (dimensionless)');
title('energies vs. time for rimless wheel');

subplot(122)
plot(ts,energies) % zoom in on energy for the last step
xlabel('time (dimensionless)');
ylabel('energy (dimensionless)');
title('energies vs. time for final step of rimless wheel');

%% Thetadot per step 
figure
plot(0:length(x0s)-1, x0s(:,2),'.')
xlabel('step number');
ylabel('initial thetadot (dimensionless)');
title('initial thetadot vs. step number');
hold on
plot([0,length(x0s)-1], thetadotplusstar*[1 1],':')
legend('thetadot', 'analytical limit cycle')


