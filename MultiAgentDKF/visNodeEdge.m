visXData = []; visYData = [];
visLineXData = []; visLineYData = [];
% visualize node
for vis_i=1:totalAgent
    visXData = [visXData agentList(vis_i).px];
    visYData = [visYData agentList(vis_i).py];
end    
% visualize proximity net's edge
for vis_i=1:totalAgent
    for vis_j=vis_i:totalAgent
        if agentNeighList(vis_i,vis_j) == 1
            %                                       location of this ;
            %                                      is very important..
            visLineXData = [visLineXData [agentList(vis_i).px; agentList(vis_j).px]];
            visLineYData = [visLineYData [agentList(vis_i).py; agentList(vis_j).py]];
        end
    end
end