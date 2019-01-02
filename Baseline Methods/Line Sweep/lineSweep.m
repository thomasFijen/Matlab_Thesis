%%  MSc Thesis
% Thomas Fijen, 4620852

function agent = lineSweep(agent, MS, grid, index)

% lineSweep      Determines the agents new heading according to line sweep     
%
% Syntax:        MAV = lineSweep(MAV, MS, grid)
%
% Inputs:               
%   MAV     -   MAV Structure
%   MS      -   2D mission space age grid
%
% Outputus:
%   []      -   None

% NOTE:
% This is a simple implementation of the line sweep method. In this I
% assume that:
%       -   Start at bottom left with heading = 0
%       -   there are no obstacles in the centre of the mission space,
%       -   only one agent is assigned to each mission space 
%       -   turning 90 deg requires one full time step
%       -   The sweep direction is assumed to be verticle 
%           (ie: heading = [0,pi])
%       -   No need for diagonal heading changes
%       -   There are no fuel constraints
%       -   Assumes area boundries are marked as no fly cells


%  if (MAV.head == 0 || MAV.head == pi) && MAV.turnStatus == 0
%     
%     %-- the next position if continues on current heading
%     tempX = MAV.posX + MAV.vel*grid.ts*sin(MAV.head);
%     tempY = MAV.posY + MAV.vel*grid.ts*cos(MAV.head);
%     
%     %-- Determining which cell (of age map) the agent will be in
%     cellX = ceil(tempX/grid.res);
%     cellY = ceil(tempY/grid.res);
%     
%     if MS(cellY,cellX) == -1 || (MS(cellY+1,cellX) == -1 && mod(tempY,grid.res) == 0) %Turn if meet boundry
%         if MAV.head == 0 %-- Check turn direction
%             MAV.turnStatus = 1; %Turn right
%         else
%             MAV.turnStatus = 2; %Turn left
%         end
%         MAV.targetX = MAV.posX + grid.res;
%         MAV.targetY = MAV.posY;
%         MAV.head = pi/2;
%         MAV.targetFlag = 1;
%         
%     else    % otherwise update MAV position
%         MAV.posX = tempX; 
%         MAV.posY = tempY;
%     end
%  else
%     if MAV.turnStatus == 1
%         MAV.head = pi;
%     else
%         MAV.head = 0;
%     end
%     MAV.turnStatus = 0;
%  end

%% -- Better line sweep implementation
% Always starts in the lower left corner.
% Either verticle or horizontal sweeps

% 
% if MAV.heading == 0
%     
% elseif MAV.heading == pi/4
% elseif MAV.heading == pi/2
% elseif MAV.heading == 3*pi/4
% end

% currentCellX = ceil(MAV.posX/grid.res);
% % currentCellY = ceil(MAV.posY/grid.res);
% 
% if MAV.sweepDir == 0
%     if MAV.head == 0
%         for i = grid.numY:-1:1
%             if MS(i,currentCellX+1) > 0
%                 MAV.targetY = grid.res/2+(i-1)*grid.res;
%                 break;
%             end%-- Selects the cell in the next row 
%         end
%         MAV.targetX = MAV.posX + grid.res;
%         
%     elseif MAV.head == pi/2
%         if MAV.posY < grid.numY/2 %-- NOTE: not ideal!!!!
%             for i = grid.numY:-1:1
%                 if MS(i,currentCellX) > 0
%                     MAV.targetY = grid.res/2+(i-1)*grid.res;
%                     break;
%                 end%-- Selects the cell in the next row 
%             end
%         else
%             for i = 1:grid.numY
%                 if MS(i,currentCellX) > 0
%                     MAV.targetY = grid.res/2+(i-1)*grid.res;
%                     break;
%                 end%-- Selects the cell in the next row 
%             end  
%         end
%         
%     elseif MAV.head == pi
%         for i = 1:grid.numY
%             if MS(i,currentCellX+1) > 0
%                 MAV.targetY = grid.res/2+(i-1)*grid.res;
%                 break;
%             end%-- Selects the cell in the next row 
%         end
%         MAV.targetX = MAV.posX + grid.res; 
%     else %-- DEFAULT ACTION
%         for i = grid.numY:-1:1
%             if MS(i,currentCellX) > 0
%                 MAV.targetY = grid.res/2+(i-1)*grid.res;
%                 break;
%             end%-- Selects the cell in the next row 
%         end    
%     end
% else
%     
% end
% MAV.targetFlag = 1;

%% -- Better line sweep implementation
% Always starts in the lower left corner of subregion.
% Either verticle or horizontal sweeps

[xIndex, yIndex] = getMS_ArrayIndex_B(agent, grid);

%--If in starting position
if abs(agent.posY - grid.start(index,2)) < 0.1 && abs(agent.posX - grid.start(index,1)) < 0.1
    numCells = floor((grid.regions(index,3) - agent.posY - grid.res/2)/grid.res);
        
    for i=(yIndex+numCells):-1:yIndex
        if MS(i,xIndex) ~= -1
            agent.targetY = grid.regions(index,3)-grid.res/2;%i*grid.res+grid.res/2;
            agent.targetX = agent.posX;
            break;
        end
    end
    agent.turnFlag = 1;
else
    %--For the case when not in the starting position
    if agent.turnFlag == 1 
        agent.targetY = agent.posY;
        agent.targetX = agent.posX + grid.res;
        
        if agent.targetX > grid.regions(index,2)
            agent.targetY = grid.start(index,2);
            agent.targetX = grid.start(index,1);
        end
        
        agent.turnFlag = 0;
    else
        agent.turnFlag = 1;
        
        if agent.posY < (grid.regions(index,4)+(grid.regions(index,3)-grid.regions(index,4))/2)
            numCells = floor((grid.regions(index,3) - agent.posY - grid.res/2)/grid.res);
%             for i=(yIndex+numCells):-1:yIndex
%                 if MS(i,xIndex) ~= -1
%                     agent.targetY = i*grid.res+grid.res/2;
%                     break;
%                 end
%             end
            agent.targetY = grid.regions(index,3)-grid.res/2;
            
        else
            numCells = floor((agent.posY - grid.regions(index,4) + grid.res/2)/grid.res);
%             for i=(yIndex-numCells):yIndex
%                 if MS(i,xIndex) ~= -1
%                     agent.targetY = i*grid.res-grid.res/2;
%                     break;
%                 end
%             end
            agent.targetY = grid.start(index,2);
        end
        agent.targetX = agent.posX;
    end
end

agent.targetFlag = 1;

end %-- End function lineSweep

