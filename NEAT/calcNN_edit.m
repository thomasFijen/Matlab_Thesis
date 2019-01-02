%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- calcNN
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to simulate the persistent sureillance mission
% under the control of the given ANN.
% Date created: 11 July 2018

function [out,pop_new] = calcNN_edit(pop,hiddenLayers,numIn,numOut,found)
% This function calculates the output of the NN for the given population

%--Defining the activation function. 
% a=1; %--This parameter defines the steepness of the activation slope, 1 is default.
% activFunc = @(x) 1/(1+exp(-a*x)); %--Sigmoid 
activFunc = @ (x) (exp(x)-exp(-x))/(exp(x)+exp(-x)); %--Hyperbolic tangent

out = zeros(numOut,1);

%--Determining the output states for any nodes in the hiddenlayers
if(hiddenLayers(1,1) ~= 0)
    nodeInLayer = size(hiddenLayers,2);
    for i=1:nodeInLayer
        
        index = find(pop.nodegenes(1,:) == hiddenLayers(1,i));
        connectionNodes = find(pop.connectiongenes(3,:) == hiddenLayers(1,i));
        
        if(size(connectionNodes,2) > 0)
            sumInput = 0;
            for j=1:size(connectionNodes,2)
                inputNode = find(pop.nodegenes(1,:) == pop.connectiongenes(2,connectionNodes(j)));
                sumInput = sumInput + pop.connectiongenes(4,connectionNodes(j))*pop.connectiongenes(5,connectionNodes(j))*pop.nodegenes(4,inputNode);
            end
            pop.nodegenes(3,index) = sumInput;
            pop.nodegenes(4,index) = activFunc(sumInput);
        else
           pop.nodegenes(3,index) = 0;
           pop.nodegenes(4,index) = 0;
        end
    end
    for i=1:numOut
        out(i) = pop.nodegenes(4,numIn+1+i);
    end
end

% %--Determining output states of the ouput nodes
% bool_interConnect = 0;
% for i=1:numOut
%     connectionNodes = find(pop.connectiongenes(3,:) == (numIn+1+i));
% 
%     if(size(connectionNodes,2) > 0)
%         sumInput = 0;
%         for j=1:size(connectionNodes,2)
%             inputNode = find(pop.nodegenes(1,:) == pop.connectiongenes(2,connectionNodes(j)));
%             sumInput = sumInput + pop.connectiongenes(4,connectionNodes(j))*pop.connectiongenes(5,connectionNodes(j))*pop.nodegenes(4,inputNode);
%         end
%         pop.nodegenes(3,numIn+1+i) = sumInput;
%         pop.nodegenes(4,numIn+1+i) = activFunc(sumInput);
%         out(i) = activFunc(sumInput);
%     else
%         out(i) = 0;
%     end
%     
%     %--Check to see if the outputs are interconneced with each other
%     if find(pop.connectiongenes(2,:) == (numIn+1+i))
%         bool_interConnect = 1;
%     end
% end

if found == 0
    num_nodes = size(pop.nodegenes,2);
    num_connections = size(pop.connectiongenes,2);
    no_change_threshold=1e-3;
    no_change_count = 0;
    index_loop = 0;         %-- Tracks the number of times the loop is run
    
%     temp2(:,:,1) = pop.nodegenes;
    
%     pop.nodegenes(4,(numIn+2):num_nodes) = (exp(pop.nodegenes(3,(numIn+2):num_nodes))-exp(-pop.nodegenes(3,(numIn+2):num_nodes)))./(exp(pop.nodegenes(3,(numIn+2):num_nodes))+exp(-pop.nodegenes(3,(numIn+2):num_nodes))); %-- HYPERBOLIC TANGENT ACTIVATION FUNCTION
    
    for i=1:numOut
        connectionNodes = find(pop.connectiongenes(3,:) == (numIn+1+i));

        if(size(connectionNodes,2) > 0)
            sumInput = 0;
            for j=1:size(connectionNodes,2)
                inputNode = find(pop.nodegenes(1,:) == pop.connectiongenes(2,connectionNodes(j)));
                sumInput = sumInput + pop.connectiongenes(4,connectionNodes(j))*pop.connectiongenes(5,connectionNodes(j))*pop.nodegenes(4,inputNode);
            end
            pop.nodegenes(3,numIn+1+i) = sumInput;
            pop.nodegenes(4,numIn+1+i) = activFunc(sumInput);
        end
    end
    
    while (no_change_count < num_nodes) && index_loop < 3*num_connections    
         index_loop = index_loop + 1;
         vector_node_state = pop.nodegenes(4,:); %-- vector containing the previous nodes output states
         
        for index_connections=1:num_connections
            %read relevant contents of connection gene (ID of Node where connection starts, ID of Node where it ends, and connection weight)
            % This is just for convenience...
            ID_connection_from_node = pop.connectiongenes(2,index_connections);
            ID_connection_to_node = pop.connectiongenes(3,index_connections);
            connection_weight = pop.connectiongenes(4,index_connections);

            %map node ID's (as extracted from single connection genes above) to index of corresponding node in node genes matrix
            index_connection_from_node = find((pop.nodegenes(1,:)==ID_connection_from_node));
            index_connection_to_node = find((pop.nodegenes(1,:)==ID_connection_to_node));

            %This performs the sumation of inputs to nodes
            if pop.connectiongenes(5,index_connections) == 1 %Check if Connection is enabled
               pop.nodegenes(3,index_connection_to_node) = pop.nodegenes(3,index_connection_to_node)+pop.nodegenes(4,index_connection_from_node)*connection_weight; %take output state of connection_from node, multiply with weight, add to input state of connection_to node
            end
        end%-- End of FOR loop

        %--pass on node input states to outputs for next itteration 
        pop.nodegenes(4,(numIn+2):num_nodes) = (exp(pop.nodegenes(3,(numIn+2):num_nodes))-exp(-pop.nodegenes(3,(numIn+2):num_nodes)))./(exp(pop.nodegenes(3,(numIn+2):num_nodes))+exp(-pop.nodegenes(3,(numIn+2):num_nodes)));

        %Re-initialize node input states for next itteration
        pop.nodegenes(3,(numIn+2):num_nodes)=0; %set all output and hidden node input states to zero
        no_change_count=sum(abs(pop.nodegenes(4,:)-vector_node_state)<no_change_threshold); %check for alle nodes where the node output state has changed by less than no_change_threshold since last iteration through all the connection genes
    
%         temp2(:,:,index_loop+1) = pop.nodegenes;
    end  %-- End of the WHILE loop 

    %--This IF statement is just for debugging/testing, tells us that
    %the generation of the outputs in the previous loops never reached
    %equilibrium
    if index_loop >= 2.7*num_connections
        disp('Warning in simulatedRun: ANN output not converging');
%         pop.connectiongenes
        pop.nodegenes
    end
    out(1) = pop.nodegenes(4,numIn+2);
    out(2) = pop.nodegenes(4,numIn+3);
end

pop_new = pop;

end

