%
% This piece of code is based on paper:
% Flocking for Multi-Agent Dynamic Systems:
% Algorithms and Theory
%


%% simulation setup
% simulation time interval
dt = 0.05;
% simulation step
totalStep = 500;
currStep = 0;

% simulation area setup
xbound = [-30;30];
ybound = [-30;30];

% agent init
agentList = [];
totalAgent = 30;
for i=1:totalAgent
    pos = 1 + 10.*randn(2,1);
    vel = 0.4.*randn(2,1);
    agentList = [agentList BasicAgent(pos, vel)];
end

% flocking relate variables (From Reza-Olifati paper Section 8)
param = {};
param.d = 7;        % interaction distance
param.r = 1.2*param.d;    % interaction range
param.dp = 0.6*param.d;
param.rp = 1.2*param.dp;
param.epsilon = 0.1;
param.a = 5; param.b = 5;
param.bumph = 0.2;

% proximity net (spatial induced graph) generation
agentNeighList = getNeighList(agentList, param);

%%
% visualization figure
fig = figure('name', 'Simulation of Agents');

visNodeEdge; % a script used to shorten code size

h = plot(visXData, visYData,'*'); hold on;
h2 = plot(visLineXData, visLineYData,'g');
axis([xbound(1) xbound(2) ybound(1) ybound(2)]);
grid on;

%% simulate free moving
fprintf('\nSimulation Counter: ')
for sim_step=1:totalStep
    fprintf('%d/%d\n', sim_step, totalStep);
    for i=1:totalAgent
        agentList(i) = agentList(i).sim(dt);
    end
     
    % neighbour update 
    agentNeighList = getNeighList(agentList, param);
    
    % control protocol TODO
    
    
    % update visualization
    visNodeEdge; % a script used to shorten code size
    set(h,'Xdata', visXData);
    set(h,'Ydata', visYData);
    delete(h2)
    h2 = plot(visLineXData, visLineYData,'g');
    % close Figure to stop the simulation
    refreshdata; 
    pause(0.001)
    %waitforbuttonpress
end