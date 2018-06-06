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
param = {};
param.d = 7;
param.r = 1.2*d;
param.dp = 0.6*d;
param.rp = 1.2*dp;
param.epsilon = 0.1;
param.phia = 5; param.phib = 5;
param.bumph = 0.2;

% visualization figure
fig = figure('name', 'Simulation of Agents');

visXData = []; visYData = [];
visLineXData = []; visLineYData = [];
for i=1:totalAgent
    visXData = [visXData agentList(i).px];
    visYData = [visYData agentList(i).py];
    if i ~= totalAgent
        % visLineXData should contain proximity net's edge
        % here I only list these lines to test basic structure
        visLineXData = [visLineXData; [agentList(i).px agentList(i+1).px]];
        visLineYData = [visLineYData; [agentList(i).py agentList(i+1).py]];
    end
end

h = plot(visXData, visYData,'*'); hold on;
h2 = plot(visLineXData, visLineYData,'g');
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
        if j ~= totalAgent
            visLineXData(j,:) = [agentList(j).px agentList(j+1).px];
            visLineYData(j,:) = [agentList(j).py agentList(j+1).py];
        end
    end
    
    set(h,'Xdata', visXData);
    set(h,'Ydata', visYData);
%     delete(h2)
%     h2 = plot(visLineXData, visLineYData);
    h2(1).XData = visLineXData(:,1);
    h2(1).YData = visLineYData(:,1);
    h2(2).XData = visLineXData(:,2);
    h2(2).YData = visLineYData(:,2);
    
    % close Figure to stop the simulation
    refreshdata; 
    pause(0.001)
    %waitforbuttonpress
end