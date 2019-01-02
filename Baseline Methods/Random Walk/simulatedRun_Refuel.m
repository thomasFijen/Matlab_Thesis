%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- simulatedRun_Refuel
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to simulate the persistent sureillance mission
% under the control of the given ANN.
% Date created: 16 August 2018
%
%
%% ----------------

function [population_updated] = simulatedRun_Refuel(MAV, grid, sim, population, MS)
% simulatedRun_Refuel         Simulates the mission over time horizon
%
% Syntax:              MS_new = simulatedRun(ts,MS_old)
%
% Inputs:               
%   agent                   -   MAV structure
%   grid                    -   parrameters of the MS 
%   sim                     -   structure containing simulation parameters
%   population              -   structure containing population info for
%                               evolution
%
% Outputs:
%   population_updated      - updated population structure with the fitness
%                               values

population_updated = population;
no_change_threshold=1e-3; % Threshold to judge if state of a node has changed significantly since last iteration
num_individuals=size(population,2);

countOutofBounds = 0;
outOfBounds = ones(sim.numAgents,1);

%-- Assigns a random starting position for the UAV. Used for all simulation
%   runs in this generation
for i=1:sim.numAgents
    MAV(i).posX = grid.res + (grid.width-grid.res*2)*rand(1,1);
    MAV(i).posY = grid.res + (grid.bredth-grid.res*2)*rand(1,1);
end

for pop_index=1:num_individuals %-- This loop runs the simulation for each of the diferent NN in the Population
%     num_nodes = size(population(pop_index).nodegenes,2);
%     num_connections = size(population(pop_index).connectiongenes,2);
    fitness = 0;
    
    %--Reinitialise the agents and the mission space
    MS_temp = MS;
    agent = MAV;
    
    %--Initialise the depot 
    depot.posX = 10;
    depot.posY = 10;
    depot.inUse = 0;
    
    %--Create the layout of the NN
    [hiddenLayers,found] = formatNN_edited(population(pop_index),sim.num_Inputs);
    for t=0:sim.ts:sim.time    % Run sim over the give time horizon
        for i=1:sim.numAgents
        
            MAV_loop = agent(i);
            
            if ~MAV_loop.crashed 
                pop = nn_inputs(MAV_loop,grid,MS_temp,population(pop_index),sim,agent,i);
                state = behaviourTree(agent,depot,i,grid,MS_temp);
                
                switch state
                    case 1
                        MAV_loop.u1 = 0;
                        MAV_loop.u2 = 0;
                        MAV_loop.workBool = 1;
                        MAV_loop.chargeBool = 0;
                        MAV_loop.waitBool = 0;
                    case 2
                        out = homing(MAV_loop,depot,sim);
                        MAV_loop.u1 = out(1);
                        MAV_loop.u2 = out(2);
                        
                        MAV_loop.workBool = 0;
                        MAV_loop.chargeBool = 0;
                        MAV_loop.waitBool = 0;
                    case 3
                        MAV_loop.u1 = 0;
                        MAV_loop.u2 = 0;
                        
                        MAV_loop.workBool = 0;
                        MAV_loop.chargeBool = 0;
                        MAV_loop.waitBool = 1;
                    case 4
                        MAV_loop.u1 = 0;
                        MAV_loop.u2 = 0;
                        
                        MAV_loop.workBool = 0;
                        MAV_loop.chargeBool = 1;
                        MAV_loop.waitBool = 0;
                    case 5
                        MAV_loop.workBool = 1;
                        MAV_loop.chargeBool = 0;
                        MAV_loop.waitBool = 0;
                        
                        %--Calculate the output of the NN
                        out = calcNN_edit(pop,hiddenLayers,sim.num_Inputs,sim.num_Outputs,found);

                        %--Calculating the value of the velocity outputs
                        if out(1) ~= 0
                            theta = atan(abs(out(2)/out(1)));
                        else
                            theta = pi/2;
                        end
                        MAV_loop.u1 = out(1)*MAV_loop.maxVel*cos(theta);
                        MAV_loop.u2 = out(2)*MAV_loop.maxVel*sin(theta);
                        if sqrt(MAV_loop.u1^2+MAV_loop.u2^2) - MAV_loop.maxVel > 0.001
                            disp('MAX Vel Exceeded')
                        end
                end
                
                %--Simulate the agents actions in the environment for the given
                %Call agentDynamics to update position
                MAV_loop = agentDynamics_D(MAV_loop, sim);
                MAV_loop = fuel(MAV_loop,sim);
                agent(i) = MAV_loop;
                
                sumCharge = sum([agent(:).chargeBool]);
                if sumCharge > 0
                    depot.inUse = sumCharge;
                else
                    depot.inUse = 0;
                end
                
            end%-- End of the if crashed statement
        end%--End of the number of agents for loop

        %Call ageMS to age the mission space
        MS_temp = ageMS_D(grid,MS_temp,agent,sim);
        
        % ------------ Determining the fitness funcion ------------
        minDistance = 3;    %--This is the threshold for when the penalty is applied given in the article
        for i=1:sim.numAgents   %--This finds the minimum distance between any two agents
            outOfBounds(i) = 1;
            for j=1:sim.numAgents
                if i ~= j
                    distance = sqrt((agent(j).posX-agent(i).posX)^2+(agent(j).posY-agent(i).posY)^2);
                    if distance < minDistance
                        minDistance = distance;
                    end
                end
            end   
            [x_index, y_index] = getMS_ArrayIndex_D(agent(i), grid);
            if agent(i).posX > grid.width || agent(i).posX < 0 || agent(i).posY < 0 || agent(i).posY > grid.bredth
                outOfBounds(i) = 0;
            elseif MS_temp(x_index, y_index) == -1
                outOfBounds(i) = 0;
            end
        end
        s = 0.1 + minDistance/3*0.9;
   
        fitness = fitness+calcTotalAge_D(MS_temp)/grid.numCells*s;
%         fitness = fitness+calcTotalAge_D(MS_temp)/grid.numCells*s*outOfBounds;
%         fitness = fitness+calcTotalAge_D(MS_temp)/grid.numCells+sqrt(MAV_loop.u1^2+MAV_loop.u2^2)*100*population(pop_index).nodegenes(3,5);
        
        %--Terminate 'runaway' simulations
        if sum(outOfBounds) ~= size(outOfBounds,1) 
            countOutofBounds = countOutofBounds + 1;
        else
            countOutofBounds = 0;
        end
        if countOutofBounds > sim.time/5
            break;
        end
     end%-- end sim time FOR loop
    
    %-- Determine and assign the individual fitness 
    fitness = fitness * 1/sim.time;
    if fitness <= 0 
        fitness = 0.5;
    end
    population_updated(pop_index).fitness = fitness;
    
end %-- End of population FOR loop
% hold off;
end %-- End of simulatedRun function

