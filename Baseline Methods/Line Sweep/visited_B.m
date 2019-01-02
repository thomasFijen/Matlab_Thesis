%%  MSc Thesis
% Thomas Fijen, 4620852

function [MS_new,tickCount_new] = visited_B(grid,MS_old,agent,tickCount_old,sim)
% visited       Determines if a cell has bee visited, updates cell ticks     
%
% Syntax:       MS = visited(MAV,MS,grid)
%
% Inputs:               
%   MAV     -   MAV Structure
%   MS      -   2D mission space age grid
%
% Outputus:
%   MS      -   Updated mission space age grid

% NOTE: This function only considers a cell visited if the position lies
% whithin the tolerance circle. Need to consider the case when the agent
% moves through the center of a cell during the time step.

% for i=1:sim.numAgents
%     if MAV(i).posX >= 0 && MAV(i).posX <= grid.width && MAV(i).posY >= 0 && MAV(i).posY <= grid.bredth
%         %-- Determining which cell (of age map) the agent will be in
%         cellX = ceil(MAV(i).posX/grid.res);
%         cellY = ceil(MAV(i).posY/grid.res);
%         
%         if MS(cellX,cellY) ~= -1
%             MS(cellX,cellY) = MS(cellX,cellY)+1;
%         end
% 
%     %     coordX = grid.res/2+(cellX-1)*grid.res;
%     %     coordY = grid.res/2+(cellY-1)*grid.res;
%     % 
%     % 
%     %     if (((MAV.posX-coordX)^2 + (MAV.posY-coordY)^2) <= grid.posTol^2) && MS(cellY,cellX) > 0
%     %         MS(cellY,cellX) = 100;
%     %     end
%     end %-- Only check the cells if the MAV is in the mission space
% end

minDist = agent(1).footprint-grid.res/2;

% %--Aging the cells without obstacles
% [row, col] = find(MS_old~=--1);
% MS_old(row,col) = MS_old(row,col) - grid.ts;



for i=1:size(MS_old,1)
    for j = 1:size(MS_old,2)
        if MS_old(i,j) ~= -1
            %--Looking for the closest agent to the cell
            distance = minDist+5;
            for agentNum=1:size(agent,2) %--Finds the agent closest to the cell in question
                temp = sqrt((agent(agentNum).posX-((j-1)*grid.res+0.5*grid.res))^2+(agent(agentNum).posY-((i-1)*grid.res+0.5*grid.res))^2);
                if temp < distance
                    distance = temp;
                end
            end
            
            %--Updating the age of the cell
            if distance < minDist
                MS_old(i,j) = 100;
                tickCount_old(i,j) = tickCount_old(i,j) + 1;
            else
                MS_old(i,j) = MS_old(i,j) - sim.ts;
            end
            if MS_old(i,j) <= 0
                MS_old(i,j) = 0;
            end
            
        end
    end
end


MS_new = MS_old;
tickCount_new = tickCount_old;
end %-- end of function visited