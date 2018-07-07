function [agentNeighList, spatialAdjacenyMtx] = getNeighList(agentList, param)
%GETNEIGHLIST get list of neighbours of every agent in the agent list
%   

len = size(agentList,2);
agentNeighList = zeros(len,len);
spatialAdjacenyMtx = zeros(len,len);

for i= 1:len
    for j = 1:len
        if i ~= j
            dist = agentDist(agentList(i), agentList(j));
            
            if dist < param.r
                agentNeighList(i,j) = 1;
                pi = [agentList(i).px;agentList(i).py];
                pj = [agentList(j).px;agentList(j).py];
                spatialAdjacenyMtx(i,j) = spatialAdjacency(pi, pj, param);
            end
        end
    end
end

end

