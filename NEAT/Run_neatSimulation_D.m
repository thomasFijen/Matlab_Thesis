%% MSc Thesis:
% Thomas Fijen, 4620852
% This is my attemp to recreate the test performed by Miguel Daurte in his
% evolution papers (see: ref 146)

%% ------------ NEAT Simulation
%   This script implements the evolved ANN controller
%   Date: 28 May 2018

clc
clear
% close all

Font_size = 18;
set(0,'DefaultAxesXGrid','on','DefaultAxesYGrid','on',...
    'DefaultAxesZGrid','on');
set(0,'defaultFigurePosition',[488 100 560*1.6 420*1.4]);
set(0,'defaultFigurePaperPositionMode','auto');


%% ----------- Setup the mission space

test = 1;   % This determines whether to run the evolutinoary code or to test generated NN. 0 = run evolution

grid_MS.width = 25; %40        % width of rectangular mission space in [m]
grid_MS.bredth  = 25; %40      % bredth of rectangular mission space in [m] (Actually height)
grid_MS.res = 1.0;             % resolution of the discretisation
grid_MS.posTol = 0.1;          % positional tolerance of the agents
grid_MS.numCells = 0;          % Number of non obstacle cells

[MS,X,Y] = initEnviron_D(grid_MS);

grid_MS.numX = size(X,2);
grid_MS.numY = size(Y,2);
grid_MS.numCells = sum(MS(:) >= 0); %This counts the number of cells with positive values (i.e: the cells without obstacles)

%--Setting the obstacle values
obs.width = 3;
obs.height = 3;

% ---------- Defining the simulation parameters
sim.numAgents = 6;
sim.inFormat = 6;
sim.num_Outputs = 2;
sim.time = 600;             % Total simulation time in [s] 
sim.ts = 0.25;               % Size of the time steps in [s]
t_crash = 800;              % Time at which an agent fails. Not used in evolution
if sim.inFormat == 1
    sim.num_Inputs = 5;
elseif sim.inFormat == 2 || sim.inFormat == 3
    sim.num_Inputs = 9;
elseif sim.inFormat == 4
    sim.num_Inputs = 13;
elseif sim.inFormat == 5 || sim.inFormat == 6 || sim.inFormat == 8 || sim.inFormat == 9
    sim.num_Inputs = 17;
elseif sim.inFormat == 7
    sim.num_Inputs = 2;
end

% for i=2:sim.numAgents+1
%     [MS(:,:,i),X,Y] = initEnviron_D(grid_MS);
% end    
% ---------- Defining the agents
for i=1:sim.numAgents
    MAV(i).posX = grid_MS.width/2+0.5*grid_MS.res;        % [m]
    MAV(i).posY = grid_MS.bredth/2+0.5*grid_MS.res;       % [m]
    MAV(i).velX = 0;                % [m/s]
    MAV(i).accX = 0;                % [m/s^2]
    MAV(i).velY = 0;                % [m/s]
    MAV(i).accY = 0;                % [m/s^2]
    MAV(i).height = 1.5;            % [m]
    MAV(i).vel = 1;                 % Commanded velocity, [m/s]
    MAV(i).head = 0;                % Commanded heading, [rad]
    MAV(i).hc = 1.5;                % Command height, [m]
    MAV(i).targetX = 1.5;           % [m]
    MAV(i).targetY = 8.5;           % [m]
    MAV(i).targetFlag = 1;          % 1 if moving to assigned target cells
    MAV(i).u1 = 0;                  % This is the first output of the NN. Corresponds to: Velocity X
    MAV(i).u2 = 0;                  % This is the second output of the NN. Coresponds to: Velocity Y
    MAV(i).sensorRange = 6.0;       % Sensor range in [m]
    MAV(i).maxVel = 0.5;            % Maximum velocity of the agent [m/s]
    MAV(i).footprint = 2.5;         % Radius of the sensor footprint [m]

    MAV(i).refuelRate = 5;         % Rate at which the UAV refuels itself while charging, [/s]
    MAV(i).workBool = 1;
    MAV(i).chargeBool = 0;          % Boolean, is MAV recharging
    MAV(i).waitBool = 0;
    MAV(i).fuel = 100;              % This represents the fuel level with 100 being fully charged and 0 as empty
    MAV(i).dischargeRate = 0.5;       % Rate at which fuel is consumed, [unit/s] 
    MAV(i).crashed = 0;             % Boolean stating whether the MAV has crashed
    MAV(i).avoiding = 0;
    MAV(i).state = 5;
end


% -------- Defining the NEAT parameters
%- NOTE:
%-  The script assumes that initially the inputs and outputs are fully
%   connected. This can be chaged in the performNeatRun function.
%-  The maximum number of generations is set to 200 in the performNeatRun 
%   function

population_size = 100;%150


%% -------- Simulation parameters
% here I define the parameters needed in the simulation of the runs
if test == 0
    performNeatRun_D(population_size,sim,MAV,grid_MS,obs)
end

%% --TEST: 



if(test ==1)
    [tickCount,X,Y] = initEnviron_D(grid_MS);
%     tickCount = tickCount-1;
%     tickCount=tickCount./2;
%     nodeVal1 = zeros(sim.time/sim.ts+1,26);
%     nodeVal2 = zeros(sim.time/sim.ts+1,26);
%     nodeVal3 = zeros(sim.time/sim.ts+1,26);
    positions = zeros(sim.time/sim.ts+1,6);
            
%     load('Results - Liverpool\neatsave - 16.mat')
%     load('Results - Alt inputs\neatsave - 5.mat')
%     load('Results - Alt inputs\best_pop - 13.mat')

%     load('best_pop.mat')
    load('neatsave.mat')
    
%     load('Results - Pure Duarte\neatsave - 12') % 8
%     load('Results - Alt inputs\neatsave - 19.mat') % 44,5
%     load('Results - Alt inputs\neatsave - 28.mat')

%     load('Results - Final Inputs\best_pop - 11.mat') % 

    %These are the best controllers for in6!!
%     load('Results - Final Inputs\neatsave - 11.mat') % 4,9,9,11,11,16,18,7,9,...Results - Alt inputs\neatsave - 19
%     load('Results - Alt inputs\neatsave - 26.mat')
%     pop_index = 75; %4,1,12,5,53,4,5,3,21,11
%     popRes = population(pop_index);
    
%     load('Results - Liverpool\neatsave - 14.mat') % 4,9,9,11,11,16,18,7,9,...Results - Alt inputs\neatsave - 19
%     pop_index = 7; %4,1,12,5,53,4,5,3,21,11
%     popRes = population(pop_index);
    
    no_change_threshold=1e-3; % Threshold to judge if state of a node has changed significantly since last iteration
    num_individuals=size(population,2);
    
    topController = zeros(5,2);
    for i=1:size(population,2)
        [minVal,index] = min(topController(:,1));
        if population(i).fitness > minVal
            topController(index,:) = [population(i).fitness i];
        end
    end
    topController
%     pop_index = 99; 

    num_nodes = size(population(pop_index).nodegenes,2);
    num_connections = size(population(pop_index).connectiongenes,2);
    fitness = 0;
    
    countOutofBounds = 0;
    outOfBounds = ones(sim.numAgents,1);

    %--Reinitialise the agents and the mission space
        for i=1:sim.numAgents
            inObs = 1;
            while inObs
                temp_indxX = grid_MS.res + (grid_MS.width-grid_MS.res*2)*rand(1,1);
                temp_indxY = grid_MS.res + (grid_MS.bredth-grid_MS.res*2)*rand(1,1);
                if MS_temp(ceil(temp_indxY/grid_MS.res),ceil(temp_indxX/grid_MS.res)) ~= -1
                    inObs = 0;
                    MAV(i).posX = temp_indxX;
                    MAV(i).posY = temp_indxY;
                end
            end
        end
        
        %-- Start in circle around the center
%     ittOffset = 5;
%     itt = 1;
%     MAV(1).posX = MAV(1).posX;
%     MAV(1).posY = MAV(1).posY + itt*ittOffset;
%     MAV(2).posX = MAV(2).posX + itt*ittOffset*cosd(45);
%     MAV(2).posY = MAV(2).posY + itt*ittOffset*sind(45);
%     MAV(3).posX = MAV(3).posX+ itt*ittOffset;
%     MAV(3).posY = MAV(3).posY;
%     MAV(4).posX = MAV(4).posX + itt*ittOffset*cosd(45);
%     MAV(4).posY = MAV(4).posY - itt*ittOffset*sind(45);
%     MAV(5).posX = MAV(5).posX;
%     MAV(5).posY = MAV(5).posY - itt*ittOffset;
%     MAV(6).posX = MAV(6).posX - itt*ittOffset*cosd(45);
%     MAV(6).posY = MAV(6).posY - itt*ittOffset*sind(45);
%     MAV(7).posX = MAV(7).posX - itt*ittOffset;
%     MAV(7).posY = MAV(7).posY;
%     MAV(8).posX = MAV(8).posX - itt*ittOffset*cosd(45);
%     MAV(8).posY = MAV(8).posY + itt*ittOffset*sind(45);

%     MAV(1).posX = MAV(1).posX+ itt*ittOffset;
%     MAV(1).posY = MAV(1).posY+0.8;
    
    MS_temp = MS;
%     MS_Global = MS(:,:,sim.numAgents);
    agent = MAV;
%     otherIndex = 1;
    
    %--Initialise the depot 
    depot.posX = 12.5;
    depot.posY = 12.5;
    depot.inUse = 0;
    
    %--Create the layout of the NN
    [hiddenLayers,found] = formatNN_edited(population(pop_index),sim.num_Inputs);

    figure()
%     subplot(2,1,1);
    plot(agent(1).posX,agent(1).posY,'ko');
    hold on
    plot(agent(2).posX,agent(2).posY,'co');
    plot(agent(3).posX,agent(3).posY,'ro');
    plot(agent(4).posX,agent(4).posY,'go');
    plot(agent(5).posX,agent(5).posY,'yo');
    plot(agent(6).posX,agent(6).posY,'bo');
%     plot(agent(7).posX,agent(7).posY,'mo');
%     plot(agent(8).posX,agent(8).posY,'b*');
    
    interUAVdist = zeros(sim.time/sim.ts,1);
    
     for t=0:sim.ts:sim.time    % Run sim over the give time horizon
        for i=1:sim.numAgents
%             MS_temp = MS(:,:,i);
            MAV_loop = agent(i);
            if ~MAV_loop.crashed
                popRes = nn_inputs(MAV_loop,grid_MS,MS_temp,popRes,sim,agent,i);
                state = behaviourTree(agent,depot,i,grid_MS,sim);
%                 state = 5; %This overwrites the output of the BT, only allowing exploration
                MAV_loop.state = state;
                switch state
                    case 1
                        [out,MAV_loop] = avoidance(MAV_loop,sim,grid_MS,depot);
                        MAV_loop.u1 = out(1);
                        MAV_loop.u2 = out(2);
                        MAV_loop.workBool = 1;
                        MAV_loop.chargeBool = 0;
                        MAV_loop.waitBool = 0;
                        disp(['Agent ',num2str(i),' Avoiding'])
                    case 2
                        out = homing(MAV_loop,depot,sim);
                        MAV_loop.u1 = out(1);
                        MAV_loop.u2 = out(2);

                        MAV_loop.avoiding = 0;
                        MAV_loop.workBool = 0;
                        MAV_loop.chargeBool = 0;
                        MAV_loop.waitBool = 0;
                    case 3
                        MAV_loop.u1 = 0;
                        MAV_loop.u2 = 0;

                        MAV_loop.avoiding = 0;
                        MAV_loop.workBool = 0;
                        MAV_loop.chargeBool = 0;
                        MAV_loop.waitBool = 1;

                        if sum([agent(:).state] == 4) == 0
                            MAV_loop.state = 4;
                            MAV_loop.chargeBool = 1;
                            MAV_loop.waitBool = 0;
                        end
                    case 4
                        MAV_loop.u1 = 0;
                        MAV_loop.u2 = 0;

                        MAV_loop.avoiding = 0;
                        MAV_loop.workBool = 0;
                        MAV_loop.chargeBool = 1;
                        MAV_loop.waitBool = 0;
                    case 5
                        MAV_loop.workBool = 1;
                        MAV_loop.chargeBool = 0;
                        MAV_loop.waitBool = 0;
                        MAV_loop.avoiding = 0;

                        %--Calculate the output of the NN
                        out = calcNN_edit(popRes,hiddenLayers,sim.num_Inputs,sim.num_Outputs,found);

                        %--Calculating the value of the velocity outputs
                        if out(1) ~= 0
                            theta = atan(abs(out(2)/out(1)));
                        else
                            theta = pi/2;
                        end
                        MAV_loop.u1 =  out(1)*MAV_loop.maxVel*cos(theta);
                        MAV_loop.u2 =  out(2)*MAV_loop.maxVel*sin(theta);
                        if sqrt(MAV_loop.u1^2+MAV_loop.u2^2) - MAV_loop.maxVel > 0.001
                            disp('MAX Vel Exceeded')
                        end
                end
                %--Simulate the agents actions in the environment for the given
                %Call agentDynamics to update position
                MAV_loop = agentDynamics_D(MAV_loop, sim);
                MAV_loop = fuel(MAV_loop,sim);      %Comment out to remove the fuel constraint
                agent(i) = MAV_loop;

                depot.charge = sum([agent(:).state] == 3)+ (sum([agent(:).state] == 4));
                depot.inUse = sim.numAgents-sum([agent(:).workBool]);

            end%-- End of the if crashed statement
        end%--End of the number of agents for loop
                    
%             positions(t/sim.ts+1,i*2-1:2*i) = [MAV_loop.posX MAV_loop.posY];
            
%             %--Update the local age map
%             if floor(t) == t
%                 MS_temp = ageMS_local(grid_MS,MS_temp,agent,sim,i,otherIndex);
%                 otherIndex = otherIndex + 1;
%                 if otherIndex > sim.numAgents
%                     otherIndex = 1;
%                 end
%             else
%                 MS_temp = ageMS_local(grid_MS,MS_temp,agent,sim,i,0);
%             end
%             
%             MS(:,:,i) = MS_temp;


        %Call ageMS to age the mission space
        [MS_temp,tickCount] = visited_D(grid_MS,MS_temp,agent,tickCount,sim);
        
        %--Storing the distance between the drones
        minDistance = 5;    %--This is the threshold for when the penalty is applied given in the article
        for i=1:sim.numAgents   %--This finds the minimum distance between any two agents
            for agentCount=1:sim.numAgents
                if i ~= agentCount
                    distance = sqrt((agent(agentCount).posX-agent(i).posX)^2+(agent(agentCount).posY-agent(i).posY)^2);
                    if distance < minDistance
                        minDistance = distance;
                    end
                end
            end   
        end
        interUAVdist(floor(t/sim.ts+1)) = minDistance;

 %--Crash agent 8 at time t_crash
            if t==t_crash
                sim.numAgents = 7;
                agent = agent(1:1);
            end        

%         figure(1)
%         subplot(2,1,1);
        plot(agent(1).posX,agent(1).posY,'ko');
        plot(agent(2).posX,agent(2).posY,'co');
        plot(agent(3).posX,agent(3).posY,'ro');
        plot(agent(4).posX,agent(4).posY,'go');
        plot(agent(5).posX,agent(5).posY,'yo');
        plot(agent(6).posX,agent(6).posY,'bo');
%         plot(agent(7).posX,agent(7).posY,'mo');
%         plot(agent(8).posX,agent(8).posY,'b*');
        
%         subplot(2,1,2);
%         mesh(MS_temp);
%         drawnow
%         pause(0.005)
        
%         %--Terminate 'runaway' simulations
%         for i=1:sim.numAgents   %--This finds the minimum distance between any two agents
%             outOfBounds(i) = 1;
%             [x_index, y_index] = getMS_ArrayIndex_D(agent(i), grid);
%             if agent(i).posX > grid.width || agent(i).posX < 0 || agent(i).posY < 0 || agent(i).posY > grid.bredth
%                 outOfBounds(i) = 0;
%             elseif MS_temp(x_index, y_index) == -1
%                 outOfBounds(i) = 0;
%             end
%         end
%     
%         if sum(outOfBounds) ~= size(outOfBounds,1) 
%             countOutofBounds = countOutofBounds + 1;
%         else
%             countOutofBounds = 0;
%         end
%         if countOutofBounds > sim.time/5
%             break;
%         end

    end%-- end sim time FOR loop
    
%     figure(1)
%     subplot(2,1,2);
%     xlabel('width of MS')
%     ylabel('Bredth of MS')
%     title('Ages of Cells in Mission Space')
%     
%     
%     subplot(2,1,1);
%     xlabel('width of MS')
%     ylabel('Bredth of MS')
%     title('Positions of UAVs')
%     legend('UAV 1','UAV 2','UAV 3','UAV 4','UAV 5','UAV 6')
% %         subplot(2,1,1);
    plot([grid_MS.width grid_MS.width],[0 grid_MS.bredth],'r');
    hold on
    plot([0 grid_MS.width],[grid_MS.bredth grid_MS.bredth],'r');
    plot([0 grid_MS.width],[0 0],'r');
    plot([0 0],[0 grid_MS.bredth],'r');
    temp = [grid_MS.res grid_MS.res;grid_MS.res (grid_MS.bredth-grid_MS.res);(grid_MS.width-grid_MS.res) (grid_MS.bredth-grid_MS.res);(grid_MS.width-grid_MS.res) grid_MS.res;grid_MS.res grid_MS.res];
    plot(temp(:,1),temp(:,2),'b')
    
    
    figure()
    plot(0:sim.ts:sim.time,interUAVdist,'ro');
%     hold on
%     plot(0:sim.ts:sim.time,interUAVdist(:,2),'bo');
%     plot(0:sim.ts:sim.time,interUAVdist(:,3),'ko');
%     legend('Between UAV 1 and 2','Between UAV 1 and 3','Between UAV 2 and 3')
%     xlabel('Time, [s]')
%     ylabel('Distance between UAVs, [m]')
%     title('Inter-drone Ranges');
    
    figure()
surface(X,Y,tickCount)
colorbar
title('Number of cell visits')
xlabel('Mission space length')
ylabel('Mission space width')
set(gca,'fontsize',Font_size)
    
%     csvwrite('NodeValues1.csv',nodeVal1);
%     csvwrite('NodeValues2.csv',nodeVal2);
%     csvwrite('NodeValues3.csv',nodeVal3);
%     csvwrite('positions.csv',positions);
end %--end if statement for testing        


