%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- ageMS
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to age the mission space by one timestep. This
% assumes that the time step is less than 1 sec and that the agents have
% the same footprint size.
% Date created: 21 February 2018
%
%
%% ----------------

function MS_new = ageMS_B(grid,MS_old,agent,sim)

% ageMS                Initialises the mission space of the assignment
%
% Syntax:              MS_new = ageMS(ts,MS_old)
%
% Inputs:               
%   ts          -   time step, [s]
%   MS_old      -   copy of the old mission space
%
% Outputus:
%   MS_new      - updated mission space

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
            if distance <= minDist
                MS_old(i,j) = 0;
            else
                MS_old(i,j) = MS_old(i,j) + sim.ts;
            end
            if MS_old(i,j) <= 0
                MS_old(i,j) = 0;
            end
            
        end
    end
end


MS_new = MS_old;

end