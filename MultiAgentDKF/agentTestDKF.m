


%% simulation setup
rng(13)
% simulation time interval
dt = 0.02;
% simulation step
totalStep = 1000;
currStep = 0;

% filtering relate variables 
param = {};
param.r = 80;
param.percept_d = 40; % within this distance, KFAgent can see gammaAgent

% simulation area setup
xbound = [-160;160];
ybound = [-160;160];

% agent init
agentList = [];
totalAgent = 20;
for i=1:totalAgent
    pos = 0 + 40.*randn(2,1);
    vel = 0.*randn(2,1);
    orient = 0;
    agentList = [agentList KFAgent(pos, vel, orient, param.percept_d, totalAgent)];
end

% a moving agent to be observed
gammaV = 10;
gammaAgent = BasicAgent([40;0], [0;-gammaV]);
gammaR = [gammaAgent.px;gammaAgent.py;0];
gammaW = [0;0;gammaV/gammaR(1)];
gammaAcc = cross(gammaW,cross(gammaW,gammaR));



% proximity net (spatial induced graph) generation
[agentNeighList, spatialAdjacenyMtx]  = getNeighList(agentList,param);

%%
% visualization figure
fig = figure('name', 'Simulation of Agents');

visNodeEdge; % a script used to shorten code size

h = plot(visXData, visYData,'*'); hold on;
h2 = plot(visLineXData, visLineYData,'g');
h3 = plot(gammaAgent.px, gammaAgent.py,'r+'); 
axis([xbound(1) xbound(2) ybound(1) ybound(2)]);
grid on;

%% simulate free moving
fprintf('\nSimulation Counter: ')
for sim_step=1:totalStep
    fprintf('%d/%d\n', sim_step, totalStep);
    
    % First simulate the movement of the moving agent
    gammaAgent = gammaAgent.sim(dt);
    gammaR = [gammaAgent.px;gammaAgent.py;0];
    gammaV = [gammaAgent.vx;gammaAgent.vy;0];
    gammaW = cross(gammaR,gammaV)/norm(gammaR)/norm(gammaR);
    gammaAcc = cross(gammaW,cross(gammaW,gammaR));
    gammaAgent = gammaAgent.setCtrl(gammaAcc(1:2));
    
    for i=1:totalAgent
        % move, then observe
        agentList(i) = agentList(i).sim(dt);
        
        agentList(i) = agentList(i).observe(gammaAgent);
    end
    
    
    % neighbour update 
    [agentNeighList, spatialAdjacenyMtx] = getNeighList(agentList,param);
    
    % update CF
    for i=1:totalAgent
        msgList = {};
        idx = 1;
        for j=1:totalAgent
            if agentNeighList(i,j) == 1
                msgList{idx} = agentList(j).getCFMsg();
                idx = idx + 1;
            end
        end
        agentList(i) = agentList(i).updateCF(msgList);
    end
    views = zeros(1, totalAgent);
    for i=1:totalAgent
        views(1,i) = agentList(i).is_in_view;
    end
    views 
    % get estimation information entropy
    states = zeros(4,totalAgent);
    for i=1:totalAgent
        states(:,i) = agentList(i).ix;
    end
    states
    e = entropy(states)
    
    % update visualization
    visNodeEdge; % a script used to shorten code size
    set(h,'Xdata', visXData);
    set(h,'Ydata', visYData);
    delete(h2)
    h2 = plot(visLineXData, visLineYData,'g');
    delete(h3)
    h3 = plot(gammaAgent.px, gammaAgent.py,'r+'); 
    % close Figure to stop the simulation
    refreshdata; 
    pause(0.001)
    %waitforbuttonpress
end