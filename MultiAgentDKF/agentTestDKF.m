


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
totalAgent = 50;
for i=1:totalAgent
    pos = 1 + 40.*randn(2,1);
    vel = 0.*randn(2,1);
    agentList = [agentList BasicAgent(pos, vel)];
end

% a moving agent to be observed
gammaAgent = BasicAgent([40;0], [0;-20]);
gammaR = [gammaAgent.px;gammaAgent.py;0];
gammaV = 440;
gammaW = [0;0;gammaV/gammaR(1)];
gammaAcc = cross(gammaW,cross(gammaW,gammaR));

% filterting relate variables 
param = {};
param.r = 40;

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
        agentList(i) = agentList(i).sim(dt);
    end
     
    % neighbour update 
    [agentNeighList, spatialAdjacenyMtx] = getNeighList(agentList,param);
    
    
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