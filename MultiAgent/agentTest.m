%% simulation setup
% simulation time interval
dt = 0.05;
% simulation step
totalStep = 500;
currStep = 0;

% simulation area setup
xbound = [-20;20];
ybound = [-20;20];

% agent init
agentList = [];
totalAgent = 50;
for i=1:totalAgent
    pos = 1 + 2.*randn(2,1);
    vel = 0.4.*randn(2,1);
    agentList = [agentList BasicAgent(pos, vel)];
end

% flocking relate variables (From Reza-Olifati paper Section 8)
d = 7;
r = 1.2*d;
dp = 0.6*d;
rp = 1.2*dp;
epsilon = 0.1;
phia = 5; phib = 5;
bumph = 0.2;

% visualization figure
fig = figure('name', 'Simulation of Agents');

visXData = []; visYData = [];
for i=1:totalAgent
    visXData = [visXData agentList(i).px];
    visYData = [visYData agentList(i).py];
end

h = plot(visXData, visYData,'*');
axis([xbound(1) xbound(2) ybound(1) ybound(2)]);
grid on;

%% simulate free moving
fprintf('\nSimulation Counter: ')
for i=1:totalStep
    fprintf('%d/%d\n', i, totalStep);
    for j=1:totalAgent
        agentList(j) = agentList(j).sim(dt);
        
        visXData(j) = agentList(j).px;
        visYData(j) = agentList(j).py;
    end
    
    set(h,'Xdata', visXData);
    set(h,'Ydata', visYData);
    refreshdata; 
    pause(0.001)
    %waitforbuttonpress
end