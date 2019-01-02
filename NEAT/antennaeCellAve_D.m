%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- antennae 
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to generate the ranges to sensed obstacles on the
% four sides of the given UAV
% Date created: 4 July 2018
%
%
%% ----------------

function [ant1_cell, ant2_cell, ant3_cell, ant4_cell, ant5_cell, ant6_cell, ant7_cell, ant8_cell ] = antennaeCellAve_D( agent,MS,grid,index )
% antennaeCell_D        This function adds sensors to the model. These sensors
%                   reach out like antennae to feel the distance and age of
%                   the cell at the 'end' of the antennae. See ref 172 for
%                   example. This is used to measure distance to the
%                   obstacles and other agents. antennae 1 points forward
%                   and the rest are numbered in a clockwise direction
%
% Syntax:           [ ant1, ant2, ant3, ant4, ant5, ant6, ant7, ant8 ] = antennae_D( agents,MS,grid )
%
% Inputs:               
%   agent           -   structure containing current agent parameters
%   grid            -   structure containing grid parameters
%   MS              -   Mission space for the simulation
%   index           -   Index of the current agent
%
% Outputus:
%   ant             -   This is the range to an obstacle
%   ant_cell        -   This is the age of the sensed cell

% NOTE: This function assumes that the agents heading does not change
% during flight. Still have to add check for the other drones. Distances
% are taken from the agent to the edge of the cell

if agent(index).posX >= 0 && agent(index).posX <= grid.width && agent(index).posY >= 0 && agent(index).posY <= grid.bredth
    %--Find the subset of cells that can be sensed
    rangeX_max = min(agent(index).sensorRange,grid.width-agent(index).posX);
    rangeX_Min = min(agent(index).sensorRange,agent(index).posX);
    rangeY_max = min(agent(index).sensorRange,grid.bredth-agent(index).posY);
    rangeY_Min = min(agent(index).sensorRange,agent(index).posY);

    indexY_s = ceil((agent(index).posY-rangeY_Min)/grid.res);
    indexY_e = ceil((agent(index).posY+rangeY_max)/grid.res);
    indexX_s = ceil((agent(index).posX-rangeX_Min)/grid.res);
    indexX_e = ceil((agent(index).posX+rangeX_max)/grid.res);

    if indexY_s == 0 
        indexY_s = 1;
    end
    if indexX_s == 0 
        indexX_s = 1;
    end
    
    [x_index, y_index] = getMS_ArrayIndex_D(agent(index), grid);
%     subsetMS = MS(indexY_s:indexY_e,indexX_s:indexX_e);
    
    %--Front sensor: 1
    i = y_index;
    ant1 = i*grid.res-agent(index).posY;
    ant1_cell = MS(y_index,x_index);
    while i < indexY_e && ant1 < agent(index).sensorRange
        i=i+1;
        if MS(i,x_index) ~= -1
            ant1 = ant1+grid.res;
            ant1_cell = ant1_cell+MS(i,x_index);
        else
            break;
        end
        if ant1 > agent(index).sensorRange
            ant1 = agent(index).sensorRange;
        end
    end
    ant1_cell = ant1_cell/(i-y_index+1);
    
    %--Right sensor: 3
    i = x_index;
    ant3 = i*grid.res-agent(index).posX;
    ant3_cell = MS(y_index,x_index);
    while i < indexX_e && ant3 < agent(index).sensorRange
        i=i+1;
        if MS(y_index,i) ~= -1
            ant3 = ant3 + grid.res;
            ant3_cell = ant3_cell+MS(y_index,i);
        else
            break;
        end
        if ant3 > agent(index).sensorRange
            ant3 = agent(index).sensorRange;
        end
    end
    ant3_cell = ant3_cell/(i-x_index+1);
    
    %--Back sensor: 5
    i = y_index;
    ant5 = agent(index).posY-(i-1)*grid.res;
    ant5_cell = MS(y_index,x_index);
    while i > indexY_s && ant5 < agent(index).sensorRange
        i=i-1;
        if MS(i,x_index) ~= -1
            ant5 = ant5 + grid.res;
            ant5_cell = ant5_cell+MS(i,x_index);
        else
            break;
        end
        if ant5 > agent(index).sensorRange
            ant5 = agent(index).sensorRange;
        end
    end
    ant5_cell=ant5_cell/(i-y_index+1);
    
    %--Left sensor: 7
    i = x_index;
    ant7 = agent(index).posX-(i-1)*grid.res;
    ant7_cell = MS(y_index,x_index);
    while i > indexX_s && ant7 < agent(index).sensorRange
        i=i-1;
        if MS(y_index,i) ~= -1
            ant7 = ant7 + grid.res;
            ant7_cell = ant5_cell+MS(y_index,i);
        else
            break;
        end
        if ant7 > agent(index).sensorRange
            ant7 = agent(index).sensorRange;
        end
    end
    ant7_cell=ant7_cell/(i-x_index+1);
    
    %--45 deg sensor: 2
    x = x_index;
    y = y_index;
    ant2 = sqrt((y*grid.res-agent(index).posY)^2+(x*grid.res-agent(index).posX)^2);
    ant2_cell = MS(y_index,x_index);
    while x < indexX_e && y < indexY_e && ant2 < agent(index).sensorRange
        x=x+1;
        y=y+1;
        if MS(y,x) ~= -1
            ant2 = ant2 + grid.res/cos(pi/4);
            ant2_cell = ant2_cell+MS(y,x);
        else
            break;
        end
        if ant2 > agent(index).sensorRange
            ant2 = agent(index).sensorRange;
        end
    end
    ant2_cell=ant2_cell/(x-x_index+1);
    
    %--135 deg sensor: 4
    x = x_index;
    y = y_index;
    ant4 = sqrt(((y-1)*grid.res-agent(index).posY)^2+(x*grid.res-agent(index).posX)^2);
    ant4_cell = MS(y_index,x_index);
    while x < indexX_e && y > indexY_s && ant4 < agent(index).sensorRange
        x=x+1;
        y=y-1;
        if MS(y,x) ~= -1
            ant4 = ant4 + grid.res/cos(pi/4);
            ant4_cell = ant4_cell+MS(y,x);
        else
            break;
        end
        if ant4 > agent(index).sensorRange
            ant4 = agent(index).sensorRange;
        end
    end
    ant4_cell=ant4_cell/(x-x_index+1);
    
    %--225 deg sensor: 6
    x = x_index;
    y = y_index;
    ant6 = sqrt(((y-1)*grid.res-agent(index).posY)^2+((x-1)*grid.res-agent(index).posX)^2);
    ant6_cell = MS(y_index,x_index);
    while x > indexX_s && y > indexY_s && ant6 < agent(index).sensorRange
        x=x-1;
        y=y-1;
        if MS(y,x) ~= -1
            ant6 = ant6 + grid.res/cos(pi/4);
            ant6_cell = ant6_cell+MS(y,x);
        else
            break;
        end
        if ant6 > agent(index).sensorRange
            ant6 = agent(index).sensorRange;
        end
    end
    ant6_cell=ant6_cell/(x-x_index+1);
    
    %--315 deg sensor: 8
    x = x_index;
    y = y_index;
    ant8 = sqrt((y*grid.res-agent(index).posY)^2+((x-1)*grid.res-agent(index).posX)^2);
    ant8_cell = MS(y_index,x_index);
    while x > indexX_s && y < indexY_e && ant8 < agent(index).sensorRange
        x=x-1;
        y=y+1;
        if MS(y,x) ~= -1
            ant8 = ant8 + grid.res/cos(pi/4);
            ant8_cell = ant8_cell+MS(y,x);
        else
            break;
        end
        if ant8 > agent(index).sensorRange
            ant8 = agent(index).sensorRange;
        end
    end
    ant8_cell=ant8_cell/(x-x_index+1);
    
%     %-- Checking the distances to the other drones
%     for i=1:size(agent,2)
%         if i ~= index
%             %--Calculating the distances and angles to the other UAVs
%             distance = sqrt((agent(index).posX-agent(i).posX)^2+(agent(index).posY-agent(i).posY)^2);
%             
%             if distance < agent(index).sensorRange
%                 [x_index2, y_index2] = getMS_ArrayIndex_D(agent(i), grid);
%                 
%                 if x_index == x_index2 && y_index2 >= y_index && distance < ant1
%                     ant1 = distance;
%                     ant1_cell = 100-MS(x_index2, y_index2);
%                 elseif (y_index2-y_index) == (x_index2-x_index) && x_index2 >= x_index && distance < ant2
%                     ant2 = distance;
%                     ant2_cell = 100-MS(x_index2, y_index2);
%                 elseif y_index == y_index2 && x_index2 >= x_index && distance < ant3
%                     ant3 = distance;
%                     ant3_cell = 100-MS(x_index2, y_index2);
%                 elseif (y_index-y_index2) == (x_index2-x_index) && x_index2 >= x_index && distance < ant4
%                     ant4 = distance;
%                     ant4_cell = 100-MS(x_index2, y_index2);
%                 elseif y_index > y_index2 && x_index == x_index2 && distance < ant5
%                     ant5 = distance;
%                     ant5_cell = 100-MS(x_index2, y_index2);
%                 elseif (y_index-y_index2) == (x_index-x_index2) && x_index2 < x_index && distance < ant6
%                     ant6 = distance;
%                     ant6_cell = 100-MS(x_index2, y_index2);
%                 elseif y_index == y_index2 && x_index2 < x_index && distance < ant7
%                     ant7 = distance;
%                     ant7_cell = 100-MS(x_index2, y_index2);
%                 elseif (y_index2-y_index) == (x_index-x_index2) && x_index2 < x_index && distance < ant8
%                     ant8 = distance;
%                     ant8_cell = 100-MS(x_index2, y_index2);
%                 end
%             end
%             
%         end
%     end
 
else
    %--For the case when the agent is ouside of the MS One of the sensors
    %will point to the MS and be given the Max cell age

    ant1_cell = 0;
    ant2_cell = 0;
    ant3_cell = 0;
    ant4_cell = 0;
    ant5_cell = 0;
    ant6_cell = 0;
    ant7_cell = 0;
    ant8_cell = 0;
    
    if agent(index).posX < 0
        if agent(index).posY < 0
            ant2_cell = 100;
        elseif agent(index).posY >= 0 && agent(index).posY <= grid.bredth
            ant3_cell = 100;
        else
            ant4_cell = 100;
        end
    elseif agent(index).posX >=0 && agent(index).posX <= grid.width
        if agent(index).posY < 0
            ant1_cell = 100;
        elseif agent(index).posY > grid.bredth
            ant5_cell = 100;
        end
    else
        if agent(index).posY < 0
            ant8_cell = 100;
        elseif agent(index).posY >= 0 && agent(index).posY <= grid.bredth
            ant7_cell = 100;
        else
            ant6_cell = 100;
        end
    end
    
end

%--normalising the values
% divisor = ant1_cell+ant2_cell+ant3_cell+ant4_cell+ant5_cell+ant6_cell+ant7_cell+ant8_cell;
% if divisor ~= 0
%     ant1_cell = ant1_cell/divisor;
%     ant2_cell = ant2_cell/divisor;
%     ant3_cell = ant3_cell/divisor;
%     ant4_cell = ant4_cell/divisor;
%     ant5_cell = ant5_cell/divisor;
%     ant6_cell = ant6_cell/divisor;
%     ant7_cell = ant7_cell/divisor;
%     ant8_cell = ant8_cell/divisor;
% end

ant1_cell = ant1_cell/100;
ant2_cell = ant2_cell/100;
ant3_cell = ant3_cell/100;
ant4_cell = ant4_cell/100;
ant5_cell = ant5_cell/100;
ant6_cell = ant6_cell/100;
ant7_cell = ant7_cell/100;
ant8_cell = ant8_cell/100;

end