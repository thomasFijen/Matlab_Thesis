%%  MSc Thesis
% Thomas Fijen, 4620852

function [ agent ] = randomWalk(MS,grid,agent,target)
% randomWalk    Implements roulette wheel based random walk     
%
% Syntax:       MAV = visited(MAV,MS,grid,target)
%
% Inputs:               
%   MAV     -   MAV Structure
%   MS      -   2D mission space age grid
%   grid    -   Structure containing details of the grid discretisation
%   target  -   Array of all agents target cells.
%
% Outputus:
%   agent   -   Updated MAV structure
%   target  -   Updated list of targets

%% -- Pure random walk

% valid = 0;
% 
% while valid == 0
%    
%     cellX = randi(size(MS,2),1,1);
%     cellY = randi(size(MS,1),1,1);
%     
%     if MS(cellY,cellX) ~= -1
%         valid = 1;
%     end
% 
% end
% 
% MAV.targetX = grid.res/2+(cellX-1)*grid.res;
% MAV.targetY = grid.res/2+(cellY-1)*grid.res;
% MAV.targetFlag = 1;

%% -- Weighted Roullette Wheel selection
%-- Adapted from: https://en.wikipedia.org/wiki/Fitness_proportionate_selection
%-- Accessed: 26-02-2018

valid = 0;              %-- Checks that chosen cell is not already target
totAge = 0;             %-- Sum of all cell ages
numX = size(MS,2);      %-- Number of cells in X direction
numY = size(MS,1);      %-- Number of cells in Y direction
lastIndex = [1 1];      %-- Index of the last non obstacle cell, [X Y]

for i = 1:numX
    for j = 1:numY
        if MS(j,i) >= 0
            totAge = totAge + MS(j,i);
            lastIndex = [i j];
        end
    end
end%-- end of 'for' finding total age (disregarding obstacle cells)


% randVal = randi(totAge,1,1);  %-- Random int value for range 0:totAge
while valid == 0
%     agent.targetFlag = 0;
%     randVal = totAge * rand(1,1);
% 
%     for i = 1:numX
%         for j = 1:numY
%             if MS(j,i) > 0
%                 randVal = randVal - (MS(j,i));
% %                 randVal = randVal - (100-MS(j,i));
%                 if randVal < 0 && agent.targetFlag == 0
%                     agent.targetX = grid.res/2+(i-1)*grid.res;
%                     agent.targetY = grid.res/2+(j-1)*grid.res;
%                     agent.targetFlag = 1;
%                 end
%                 if randVal < 0 %-- terminating the inner loop
%                     break;
%                 end
%             end%-- do not consider obstacle cells
%         end
%         if randVal < 0 %-- terminating the outer loop
%             break;
%         end
%     end
% 
%     if agent.targetFlag == 0
%         agent.targetX = grid.res/2+(lastIndex(1)-1)*grid.res;
%         agent.targetY = grid.res/2+(lastIndex(2)-1)*grid.res;
%         agent.targetFlag = 1;    
%     end%-- This accounts for rounding errors, just return the last cell
%     
    
    agent.targetX = randi(numX,1,1);
    agent.targetY = randi(numY,1,1);
    agent.targetFlag = 1;
    %-- Checking that the chosen cell is valid
    valid = 1;
    for i=1:size(target,1)
        if agent.targetX == target(i,1) && agent.targetY == target(i,2)
            valid = 0;
        end
    end
    if (MS(agent.targetY,agent.targetX) == -1)
        valid = 0;
    end

    
end% -- end of while loop that checks validity of chosen target.


end%-- end function randomWalk

