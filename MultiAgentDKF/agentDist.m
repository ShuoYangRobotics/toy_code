function d = agentDist(agent1, agent2)
%AGENTDIST get distance between two agents
%   agent1 and agent2 must be BasicAgent class

pos1 = [agent1.px; agent1.py];
pos2 = [agent2.px; agent2.py];

d = norm(pos1-pos2);

end

