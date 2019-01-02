%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- liverpool_Inputs
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is used to determine the inputs to the NN for the 
% experiments that try to recreate the data of the Liverpool students
% [ref 177].
% Date created: 30 July 2018
%
%
%% ----------------

function [dist] = liverpool_Inputs(agent, MS, agent_index,grid)
% liverpool_Inputs      Determines the two distances used by the Liverpool
%                       students in their experiments. The first is the 
%                       distance to the nearest wall and the second
%                       is the distance to the nearest UAV.
%
% Syntax:               [dist] = liverpool_Inputs(MAV, MS, agent_index)
%
% Inputs:               
%   agent               -   array of the MAV structures
%   grid                -   parrameters of the MS
%   agent_index         -   Index of the current agent
%
% Outputs:
%   dist                - two distances used for the NN input
    
dist = [0 0];

%--Distance to nearest Obstacle
if agent(agent_index).posX >= 0 && agent(agent_index).posX <= grid.width && agent(agent_index).posY >= 0 && agent(agent_index).posY <= grid.bredth %If agent is in the MS
    rangeX_max = min(agent(agent_index).sensorRange,grid.width-agent(agent_index).posX);
    rangeX_Min = min(agent(agent_index).sensorRange,agent(agent_index).posX);
    rangeY_max = min(agent(agent_index).sensorRange,grid.bredth-agent(agent_index).posY);
    rangeY_Min = min(agent(agent_index).sensorRange,agent(agent_index).posY);

    %--Finding the subset of sensed cells
    indexY_s = ceil((agent(agent_index).posY-rangeY_Min)/grid.res);
    indexY_e = ceil((agent(agent_index).posY+rangeY_max)/grid.res);
    indexX_s = ceil((agent(agent_index).posX-rangeX_Min)/grid.res);
    indexX_e = ceil((agent(agent_index).posX+rangeX_max)/grid.res);

    if indexY_s == 0 
        indexY_s = 1;
    end
    if indexX_s == 0 
        indexX_s = 1;
    end

    subsetMS = MS(indexY_s:indexY_e,indexX_s:indexX_e);

    %--Find coordinates of the obsacles in the sensed region
    [obs_Y, obs_X] = find(subsetMS == -1);
    
    %--Default range
    dist(1) = min([agent(agent_index).sensorRange,rangeX_max,rangeX_Min,rangeY_max,rangeY_Min]);
    
    if (~isempty(obs_X))
        startX = (indexX_s*grid.res-grid.res); %X pos of bottom corner of starting cell of subset MS
        startY = (indexY_s*grid.res-grid.res); %Y pos of bottom corner of starting cell of subset MS
        
        for i=1:size(obs_X,1)
            %--Determining the distance and angle to the obstacles
            obs_PosX = startX + obs_X(i)*grid.res-0.5*grid.res; %X position of the identified cell in terms of the global coordinate system
            obs_PosY = startY + obs_Y(i)*grid.res-0.5*grid.res; %Y position of the identified cell in terms of the global coordinate system
            temp_dist =  sqrt((obs_PosX-agent(agent_index).posX)^2+(obs_PosY-agent(agent_index).posY)^2);
            
            dist(1) = min(dist(1),temp_dist);
        end
    end
    
else
    dist(1) = 0;
end

%--Distance to nearest Drone
dist(2) = agent(agent_index).sensorRange;
for i=1:size(agent,2)
    if i~=agent_index
        temp_dist = sqrt((agent(i).posX-agent(agent_index).posX)^2+(agent(i).posY-agent(agent_index).posY)^2);
        
        dist(2) = min(dist(2),temp_dist);
    end
end

%--Normalise the distances
dist(1) = dist(1)/agent(agent_index).sensorRange;
dist(2) = dist(2)/agent(agent_index).sensorRange;

end