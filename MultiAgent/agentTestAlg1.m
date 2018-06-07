%
% This piece of code is based on paper:
% Flocking for Multi-Agent Dynamic Systems:
% Algorithms and Theory
% It implements Algorithm 1 in the paper, but I made a few small
% modifications to make sure the majority of agents can form a flock
% inplace


%% simulation setup
% simulation time interval
dt = 0.02;
% simulation step
totalStep = 500;
currStep = 0;

% simulation area setup
xbound = [-160;160];
ybound = [-160;160];

% agent init
agentList = [];
totalAgent = 20;
for i=1:totalAgent
    pos = 1 + 3.*randn(2,1);
    vel = -2 + 3.*randn(2,1);
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
[agentNeighList, spatialAdjacenyMtx]  = getNeighList(agentList, param);

%%
% visualization figure
fig = figure('name', 'Simulation of Agents running Algorithm 1 protocol');

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
    [agentNeighList, spatialAdjacenyMtx] = getNeighList(agentList, param);
    
    % control protocol Algorithm 1 with slight modification
    for i=1:totalAgent
        u = [0;0];
        for j=1:totalAgent
            if agentNeighList(i,j) == 1
                pos_i = [agentList(i).px;agentList(i).py];
                pos_j = [agentList(j).px;agentList(j).py];
                vel_i = [agentList(i).vx;agentList(i).vy];
                vel_j = [agentList(j).vx;agentList(j).vy];
                

                ppa = deltaNorm((pos_j-pos_i), param);
                u = u + actionFuc(ppa, param)*gradientDeltaNorm((pos_j-pos_i), param);
                u = u + spatialAdjacenyMtx(i,j)*vel_j-vel_i;
            end
        end
        agentList(i) = agentList(i).setCtrl(u);
    end
    
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