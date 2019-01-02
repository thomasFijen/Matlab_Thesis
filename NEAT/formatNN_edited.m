%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- calcNN
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to simulate the persistent sureillance mission
% under the control of the given ANN.
% Date created: 11 July 2018

function [hiddenLayers,found] = formatNN_edited(pop,numIn)
% This function calculates the format for the hidden layers of the given
% population. The output layer is considred as part of the hidden layers in
% this function.

%--Case for hidden layers
startID = numIn+2;
numNodes = startID;
maxNodes = size(pop.nodegenes,2);
hiddenLayers = zeros(2,maxNodes-numIn-1); % Row 1 = node ID, row 2 = layer number
layer_num = 1;
found = 1;

placedNodes = 1:numIn+1;
count=0;

while numNodes <= maxNodes && count <= maxNodes
    for i=startID:maxNodes
        %--ignore nodes that have aleady been placed
        if ismember(pop.nodegenes(1,i),placedNodes) == 0
            %--Find the nodes that connect to the current node, check their layer
            connectionNodes = find(pop.connectiongenes(3,:) == pop.nodegenes(1,i));
            bool_layer = 1;
            for j=1:size(connectionNodes,2)
                if ~ismember(pop.connectiongenes(2,connectionNodes(j)),placedNodes) 
                    bool_layer = 0;
                end
            end

            %--Adding the node to the layer if needed
            if bool_layer
                hiddenLayers(:,numNodes-startID+1) = [pop.nodegenes(1,i),layer_num];
                numNodes = numNodes+1;
            end
        end
    end
    placedNodes = cat(2,placedNodes,hiddenLayers(1,:));
    layer_num = layer_num+1;
    count=count+1;
end
    if(count > maxNodes)
%         disp(['WARNING: Structure of NN not Found - ',num2str(pop_index)])
        found = 0;
    end
 


end%--End of formatNN function

