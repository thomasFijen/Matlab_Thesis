%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- simulatedRun
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to update the inputs to the NN.
% Date created: 18 July 2018
%
%
%% ----------------

function [ pop_updated ] = nn_inputs( MAV_loop,grid,MS_temp,population,sim,agent,index )
% Input Formats:
%       1 - Single agent pure Duarte. 5 in, 2 out.
%       2 - Multi agent pure Duarte. 9 in, 2 out.
%       3 - Single agent, added weighted average MS ages. 9 in, 2 out.
%       4 - Multi agent, added weighted average MS ages. 13 in, 2 out.
%       5 - Multi agent, Duarte with anntenna cell values. 17 in, 2 out
%       6 - Multi agent, anntenna range and cell values. 17 in, 2 out
%       7 - Multi-agent, New neat paper replica [ref 177].2 in 2 out
%       8 - Fixed Multi agent, Duarte with anntenna cell values. 17 in, 2 out
%       9 - 

inputFormat = sim.inFormat;

num_nodes = size(population.nodegenes,2);
% num_connections = size(population(pop_index).connectiongenes,2);

%-- Simulating the range sensors. Defines the sensed MS. 
[x_index, y_index] = getMS_ArrayIndex_D(MAV_loop, grid);
% [geofence_Front,geofence_back,geofence_right,geofence_left] = geofence( MS_temp,MAV_loop,grid );

switch inputFormat
    case 1
        %--Initialise the .nodegene inputs: 5 inputs and 2 outputs 
        %For the .nodegenes, the input nodes are assumed to be 4x geofence 
        % (front, back, left, right) and boolean in area.
        % Next there is one bias node followed by two
        % output nodes (X and Y velocity). The rest are hidden nodes.
        
        [geofence_Front,geofence_back,geofence_right,geofence_left] = geofence( MS_temp,MAV_loop,grid );
        
        population.nodegenes(3,(sim.num_Inputs+2):num_nodes) = 0; %set all node input states to zero
        population.nodegenes(3,(sim.num_Inputs+1)) = 1; %bias node input state set to 1

        population.nodegenes(3,1:4) = [geofence_Front geofence_back geofence_left geofence_right];
        
    case 2
        %--Initialise the .nodegene inputs: 9 inputs and 2 outputs. PURE DUARTE!!!!!!!! 
        %For the .nodegenes, the input nodes are assumed to be:
        % 4x geofence (front, back, left, right),
        % 4x UAV range sensor (front, back, left, right)
        %and boolean in area. Next there is one bias node followed by two
        %output nodes (X and Y velocity). The rest are hidden nodes.

        population.nodegenes(3,(sim.num_Inputs+2):num_nodes) = 0; %set all node input states to zero
        population.nodegenes(3,(sim.num_Inputs+1)) = 1; %bias node input state set to 1

        %--Assigning the inpts to the NN
        [geofence_Front,geofence_back,geofence_right,geofence_left] = geofence( MS_temp,MAV_loop,grid );
        [uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left] = interDroneRange(agent,index);
        population.nodegenes(3,1:8) = [geofence_Front geofence_back geofence_left geofence_right,uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left];
    case 3
        %--Initialise the .nodegene inputs: 9 inputs and 2 outputs 
        %For the .nodegenes, the input nodes are assumed to be:
        % 4x geofence (front, back, left, right),4x weighted sum of cell ages
        %and boolean in area. Next there is one bias node followed by two
        %output nodes (X and Y velocity). The rest are hidden nodes.
        
        [geofence_Front,geofence_back,geofence_right,geofence_left] = geofence( MS_temp,MAV_loop,grid );
        
        population.nodegenes(3,(sim.num_Inputs+2):num_nodes) = 0; %set all node input states to zero
        population.nodegenes(3,(sim.num_Inputs+1)) = 1; %bias node input state set to 1

        %--Assigning the inpts to the NN
        [topLeft, topRight,bottomLeft,bottomRight] = weightedAveMS_D( grid,MS_temp,MAV_loop);
        population.nodegenes(3,1:8) = [geofence_Front geofence_back geofence_left geofence_right,topLeft, topRight,bottomLeft,bottomRight];
    case 4
        %--Initialise the .nodegene inputs: 13 inputs and 2 outputs 
        %For the .nodegenes, the input nodes are assumed to be:
        % 4x geofence (front, back, left, right),4x weighted sum of cell ages
        % 4x UAV range sensor (front, back, left, right)
        %and boolean in area. Next there is one bias node followed by two
        %output nodes (X and Y velocity). The rest are hidden nodes.
        
        population.nodegenes(3,(sim.num_Inputs+2):num_nodes) = 0; %set all node input states to zero
        population.nodegenes(3,(sim.num_Inputs+1)) = 1; %bias node input state set to 1

        %--Assigning the inpts to the NN
        [geofence_Front,geofence_back,geofence_right,geofence_left] = geofence( MS_temp,MAV_loop,grid );
        [topLeft, topRight,bottomLeft,bottomRight] = weightedAveMS_D( grid,MS_temp,MAV_loop);
        [uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left] = interDroneRange(agent,index);
        population.nodegenes(3,1:12) = [geofence_Front geofence_back geofence_left geofence_right,topLeft, topRight,bottomLeft,bottomRight,uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left];

    case 5
        %--Initialise the .nodegene inputs: 17 inputs and 2 outputs.  
        %For the .nodegenes, the input nodes are assumed to be:
        % 4x geofence (front, back, left, right),
        % 4x UAV range sensor (front, back, left, right)
        % 8x antena range values
        %and boolean in area. Next there is one bias node followed by two
        %output nodes (X and Y velocity). The rest are hidden nodes.

        population.nodegenes(3,(sim.num_Inputs+2):num_nodes) = 0; %set all node input states to zero
        population.nodegenes(3,(sim.num_Inputs+1)) = 1; %bias node input state set to 1

        %--Assigning the inpts to the NN
        [geofence_Front,geofence_back,geofence_right,geofence_left] = geofence( MS_temp,MAV_loop,grid );
        [tt1, tt2, tt3, tt4, tt5, tt6, tt7, tt8] = antennae_D( agent,MS_temp,grid,index );
%         [tt1, tt2, tt3, tt4, tt5, tt6, tt7, tt8] = antennaeCell_D( agent,MS_temp,grid,index );
        [uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left] = interDroneRange(agent,index);
        population.nodegenes(3,1:16) = [geofence_Front geofence_back geofence_left geofence_right,uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left, tt1, tt2, tt3, tt4, tt5, tt6, tt7, tt8];

    case 6
        %--Initialise the .nodegene inputs: 17 inputs and 2 outputs. Use this for the Antennae function 
        %For the .nodegenes, the input nodes are assumed to be:
        % 16x antena range and cell values
        %and boolean in area. Next there is one bias node followed by two
        %output nodes (X and Y velocity). The rest are hidden nodes.

        population.nodegenes(3,(sim.num_Inputs+2):num_nodes) = 0; %set all node input states to zero
        population.nodegenes(3,(sim.num_Inputs+1)) = 1; %bias node input state set to 1

        %--Assigning the inpts to the NN
        [t1,t2,t3,t4,t5,t6,t7,t8,tt1, tt2, tt3, tt4, tt5, tt6, tt7, tt8] = antennae_D( agent,MS_temp,grid,index );
        population.nodegenes(3,1:16) = [t1,t2,t3,t4,t5,t6,t7,t8,tt1, tt2, tt3, tt4, tt5, tt6, tt7, tt8];
    case 7
        %--Initialise the .nodegene inputs: 2 inputs and 2 outputs.  
        %For the .nodegenes, the input nodes are assumed to be:
        % 1x distance to nearest obstacle, 1x distance to nearest drone. 
        % Next there is one bias node followed by two
        % output nodes (X and Y velocity). The rest are hidden nodes.

        population.nodegenes(3,(sim.num_Inputs+2):num_nodes) = 0; %set all node input states to zero
        population.nodegenes(3,(sim.num_Inputs+1)) = 1; %bias node input state set to 1

        %--Assigning the inpts to the NN
        dist = liverpool_Inputs(agent, MS_temp, index,grid);
        population.nodegenes(3,1:2) = dist;
    case 8
        %--Initialise the .nodegene inputs: 17 inputs and 2 outputs.  
        %For the .nodegenes, the input nodes are assumed to be:
        % 4x geofence (front, back, left, right),
        % 4x UAV range sensor (front, back, left, right)
        % 8x antena cell values
        %and boolean in area. Next there is one bias node followed by two
        %output nodes (X and Y velocity). The rest are hidden nodes.

        population.nodegenes(3,(sim.num_Inputs+2):num_nodes) = 0; %set all node input states to zero
        population.nodegenes(3,(sim.num_Inputs+1)) = 1; %bias node input state set to 1

        %--Assigning the inpts to the NN
        [geofence_Front,geofence_back,geofence_right,geofence_left] = geofence( MS_temp,MAV_loop,grid );
        [tt1, tt2, tt3, tt4, tt5, tt6, tt7, tt8] = antennaeCell_D( agent,MS_temp,grid,index );
        [uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left] = interDroneRange(agent,index);
        population.nodegenes(3,1:16) = [geofence_Front geofence_back geofence_left geofence_right,uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left, tt1, tt2, tt3, tt4, tt5, tt6, tt7, tt8];
    
    case 9
        %--Initialise the .nodegene inputs: 17 inputs and 2 outputs.  
        %For the .nodegenes, the input nodes are assumed to be:
        % 4x geofence (front, back, left, right),
        % 4x UAV range sensor (front, back, left, right)
        % 8x antena cell values
        %and boolean in area. Next there is one bias node followed by two
        %output nodes (X and Y velocity). The rest are hidden nodes.

        population.nodegenes(3,(sim.num_Inputs+2):num_nodes) = 0; %set all node input states to zero
        population.nodegenes(3,(sim.num_Inputs+1)) = 1; %bias node input state set to 1

        %--Assigning the inpts to the NN
        [geofence_Front,geofence_back,geofence_right,geofence_left] = geofence( MS_temp,MAV_loop,grid );
        [tt1, tt2, tt3, tt4, tt5, tt6, tt7, tt8] = antennaeCellAve_D( agent,MS_temp,grid,index );
        [uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left] = interDroneRange(agent,index);
        population.nodegenes(3,1:16) = [geofence_Front geofence_back geofence_left geofence_right,uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left, tt1, tt2, tt3, tt4, tt5, tt6, tt7, tt8];
end

%-- boolean showing if agent is in the Mission space
if inputFormat ~= 7
    if x_index <= 0 || y_index <=0 || x_index > grid.width/grid.res || y_index > grid.bredth/grid.res
        population.nodegenes(3,sim.num_Inputs) = 0;
    else
    %     [x_index, y_index] = getMS_ArrayIndex_D(MAV_loop, grid);
        if MS_temp(y_index,x_index) == -1
            population.nodegenes(3,sim.num_Inputs) = 0;
        else
            population.nodegenes(3,sim.num_Inputs) = 1;
        end
    end %-- end IF in mission space
end

%--Initialise .nodegene output states for input and bias nodes
population.nodegenes(4,1:(sim.num_Inputs+1)) = population.nodegenes(3,1:(sim.num_Inputs+1)); %-- output of input and bias nodes are the same as their inputs

pop_updated = population;

end

