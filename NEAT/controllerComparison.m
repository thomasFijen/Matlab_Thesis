%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- controllerComparison
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to analyse and compare the performance of
% different controllers to one another. 
% Date created: 6 March 2018
%
%
clc
clear

for fuckThis=1:3

% clc
% clear
close all
andThis = [4,6,8];
%% --Initialise parameters


numLoad = 4;    % The number of test files/poulations that you want to load
startIndx = 13;
numItterations = 25; %--Number of test with current controller to perform
ittOffset = 1;      %--Change in start position of the MAVs, [m]

grid_MS.width = 25; %40       % width of rectangular mission space in [m]
grid_MS.bredth  = 25; %40     % bredth of rectangular mission space in [m] (Actually height)
grid_MS.res = 1.0;           % resolution of the discretisation
grid_MS.posTol = 0.1;      % positional tolerance of the agents
% grid.ts = 0.5;          % Size of the time steps in [s]
grid_MS.numCells = 0;

[MS,X,Y] = initEnviron_D(grid_MS);
grid_MS.numX = size(X,2);
grid_MS.numY = size(Y,2);
grid_MS.numCells = sum(MS(:) >= 0); %This counts the number of cells with positive values (i.e: the cells without obstacles)

% ---------- Defining the simulation parameters
sim.numAgents = andThis(fuckThis);
sim.inFormat = 6;
sim.num_Outputs = 2;
sim.time = 300;          % Total simulation time in [s] 
sim.ts = 0.25;
if sim.inFormat == 1
    sim.num_Inputs = 5;
elseif sim.inFormat == 2 || sim.inFormat == 3
    sim.num_Inputs = 9;
elseif sim.inFormat == 4
    sim.num_Inputs = 13;
elseif sim.inFormat == 5 || sim.inFormat == 6 || sim.inFormat == 8
    sim.num_Inputs = 17;
elseif sim.inFormat == 7
    sim.num_Inputs = 2;
end    

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
    MAV(i).u1 = 0;                  %This is the first output of the NN. Corresponds to: Velocity X
    MAV(i).u2 = 0;                  % This is the second output of the NN. Coresponds to: Velocity Y
    MAV(i).sensorRange = 6;        % Sensor range in [m]
    MAV(i).maxVel = 0.5;            % Maximum velocity of the agent [m/s]
    MAV(i).footprint = 2.5;         % Radius of the sensor footprint [m]
   
    MAV(i).refuelRate = 10;         % Rate at which the UAV refuels itself while charging, [/s]
    MAV(i).workBool = 1;
    MAV(i).chargeBool = 0;          % Boolean, is MAV recharging
    MAV(i).waitBool = 0;
    MAV(i).fuel = 100;              % This represents the fuel level with 100 being fully charged and 0 as empty
    MAV(i).dischargeRate = 1;       % Rate at which fuel is consumed, [unit/s] 
    MAV(i).crashed = 0;             % Boolean stating whether the MAV has crashed
end

%% -- Comparing controllers from same inputs
run_test =1;
if run_test == 1

%--Reading and storing the populations from the test files
for i=1:numLoad
%     fileName = ['Results - Pure Duarte\best_pop - ',num2str(startIndx+i)];
%     fileName = ['Results - Pure Duarte\neatsave - ',num2str(startIndx+i)];
    fileName = ['Results - Liverpool\neatsave - ',num2str(startIndx+i)];
%     fileName = ['Results - Alt inputs\neatsave - ',num2str(startIndx+i)];
    load(fileName,'population')
    
    test(i).pop = population;
end

%--Reading and storing the populations from the test files

% for i=1:numLoad
% %     fileName = ['Results - Alt inputs\best_pop - ',num2str(startIndx+i)];
%     fileName = ['Results - Alt inputs\neatsave - ',num2str(startIndx+i)];
%     load(fileName,'population')
%     
%     test(i).pop = population;
% end

% %--Reading and storing the populations from the test files
% for i=1:numLoad
% %     fileName = ['Results - Liverpool\best_pop - ',num2str(startIndx+i)];
%     fileName = ['Results - Liverpool\neatsave - ',num2str(startIndx+i)];
%     load(fileName,'population')
%     
%     test(i).pop = population;
% end

% %--Reading and storing the populations from the test files
% for i=1:numLoad
% %     fileName = ['Results - Refuel evo\best_pop - ',num2str(startIndx+i)];
%     fileName = ['Results - Refuel evo\neatsave - ',num2str(startIndx+i)];
%     load(fileName,'population')
%     
%     test(i).pop = population;
% end

% for i=1:numLoad
% %     fileName = ['Results - Alt inputs\best_pop - ',num2str(startIndx+i)];
%     fileName = ['Results - Final Inputs\neatsave - ',num2str(startIndx+i)];
%     load(fileName,'population')
%     
%     test(i).pop = population;
% end

reportAge = zeros(20,sim.time/sim.ts+1);
reportAgeSTD = zeros(20,sim.time/sim.ts+1);
reportCOV = zeros(20,sim.time/sim.ts+1);
reportCrash = zeros(20,numItterations);

for j=8:8
    
    %--Finding the 5 controllers with the best fitness from current test
    topController = zeros(5,2);
    for i=1:size(test(j).pop,2)
        [minVal,index] = min(topController(:,1));
        if test(j).pop(i).fitness > minVal
            topController(index,:) = [test(j).pop(i).fitness i];
        end
    end
    
%     figure(j)
    %--re-initialise performace indices
        aveAge = zeros(5,sim.time/sim.ts+1);           %--Average cell age over time
        ageSTD = zeros(5,sim.time/sim.ts+1);
        coveragePerc = zeros(5,sim.time/sim.ts+1);                      %--Percentage of area covered
        crash = zeros(5,numItterations);
        
    for contr=1:5
        %--Performance indices for the different itterations of same
        %controller
        cntrl.aveAge = zeros(numItterations,sim.time/sim.ts+1);
        cntrl.coveragePerc = zeros(numItterations,sim.time/sim.ts+1);
        cntrl.crashed = zeros(numItterations,sim.time/sim.ts+1);
        
        %--Create the layout of the NN
        [hiddenLayers,found] = formatNN_edited(test(j).pop(topController(contr,2)),sim.num_Inputs);
            
        %--Run the Simulation for n itterations
        for itt=1:numItterations
            [tickCount,X,Y] = initEnviron_D(grid_MS);
%             tickCount = tickCount-1;
%             tickCount=tickCount./2;

            %--Reinitialise the agents and the mission space
            MS_temp = MS;
            agent = MAV;
            %--Initialise the depot 
            depot.posX = 12.5;
            depot.posY = 12.5;
            depot.inUse = 0;
            
            %--Change starting Position so that for each itteration the
            %MAVs are slighlty further away from the center of the MS            
%             switch sim.numAgents
%                 case 2 
%                     agent(1).posX = agent(1).posX + itt*ittOffset;
%                     agent(1).posY = agent(1).posY + itt*ittOffset;
%                     agent(2).posX = agent(2).posX - itt*ittOffset;
%                     agent(2).posY = agent(2).posY - itt*ittOffset;
%                 case 3
%                     agent(1).posX = agent(1).posX;
%                     agent(1).posY = agent(1).posY + itt*ittOffset;
%                     agent(2).posX = agent(2).posX + itt*ittOffset;
%                     agent(2).posY = agent(2).posY - itt*ittOffset;
%                     agent(3).posX = agent(3).posX - itt*ittOffset;
%                     agent(3).posY = agent(3).posY - itt*ittOffset;
%                 case 4
%                     agent(1).posX = agent(1).posX;
%                     agent(1).posY = agent(1).posY + itt*ittOffset;
%                     agent(2).posX = agent(2).posX+ itt*ittOffset;
%                     agent(2).posY = agent(2).posY;
%                     agent(3).posX = agent(3).posX;
%                     agent(3).posY = agent(3).posY - itt*ittOffset;
%                     agent(4).posX = agent(4).posX - itt*ittOffset;
%                     agent(4).posY = agent(4).posY;
%                 case 6
%                     agent(1).posX = agent(1).posX;
%                     agent(1).posY = agent(1).posY + itt*ittOffset;
%                     agent(2).posX = agent(2).posX+ itt*ittOffset;
%                     agent(2).posY = agent(2).posY;
%                     agent(3).posX = agent(3).posX;
%                     agent(3).posY = agent(3).posY - itt*ittOffset;
%                     agent(4).posX = agent(4).posX - itt*ittOffset;
%                     agent(4).posY = agent(4).posY;
%                     agent(5).posX = agent(5).posX + itt*ittOffset*cosd(45);
%                     agent(5).posY = agent(5).posY + itt*ittOffset*sind(45);
%                     agent(6).posX = agent(6).posX - itt*ittOffset*cosd(45);
%                     agent(6).posY = agent(6).posY + itt*ittOffset*sind(45);                    
%                 case 8
%                     agent(1).posX = agent(1).posX;
%                     agent(1).posY = agent(1).posY + itt*ittOffset;
%                     agent(2).posX = agent(2).posX + itt*ittOffset*cosd(45);
%                     agent(2).posY = agent(2).posY + itt*ittOffset*sind(45);
%                     agent(3).posX = agent(3).posX+ itt*ittOffset;
%                     agent(3).posY = agent(3).posY;
%                     agent(4).posX = agent(4).posX + itt*ittOffset*cosd(45);
%                     agent(4).posY = agent(4).posY - itt*ittOffset*sind(45);
%                     agent(5).posX = agent(5).posX;
%                     agent(5).posY = agent(5).posY - itt*ittOffset;
%                     agent(6).posX = agent(6).posX - itt*ittOffset*cosd(45);
%                     agent(6).posY = agent(6).posY - itt*ittOffset*sind(45);
%                     agent(7).posX = agent(7).posX - itt*ittOffset;
%                     agent(7).posY = agent(7).posY;
%                     agent(8).posX = agent(8).posX - itt*ittOffset*cosd(45);
%                     agent(8).posY = agent(8).posY + itt*ittOffset*sind(45);
%             end
            for i=1:sim.numAgents
                inObs = 1;
                while inObs
                    temp_indxX = grid_MS.res + (grid_MS.width-grid_MS.res*2)*rand(1,1);
                    temp_indxY = grid_MS.res + (grid_MS.bredth-grid_MS.res*2)*rand(1,1);
                    if MS_temp(ceil(temp_indxY/grid_MS.res),ceil(temp_indxX/grid_MS.res)) ~= -1
                        inObs = 0;
                        agent(i).posX = temp_indxX;
                        agent(i).posY = temp_indxY;
                    end
                end
            end            


            for t=0:sim.ts:sim.time    % Run sim over the give time horizon
                for i=1:sim.numAgents
                    MAV_loop = agent(i);

                    if ~MAV_loop.crashed 
                        if sim.inFormat == 7
                            %--Assigne the inputs to the populations
                            test(j).pop(topController(contr,2)) = nn_inputs(MAV_loop,grid_MS,MS_temp,test(j).pop(topController(contr,2)),sim,agent,i);

                            %--Calculate the output of the NN
                            out = calcNN_edit(test(j).pop(topController(contr,2)),hiddenLayers,sim.num_Inputs,sim.num_Outputs,found);
                            
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

                            %--Simulate the agents actions in the environment for the given
                            %Call agentDynamics to update position
                            MAV_loop = agentDynamics_D(MAV_loop, sim);
                            agent(i) = MAV_loop;
                        else
                            test(j).pop(topController(contr,2)) = nn_inputs(MAV_loop,grid_MS,MS_temp,test(j).pop(topController(contr,2)),sim,agent,i);
                            if MAV_loop.posX <= grid_MS.width && MAV_loop.posX >= 0 && MAV_loop.posY <= grid_MS.bredth && MAV_loop.posY >=0
                                out = calcNN_edit(test(j).pop(topController(contr,2)),hiddenLayers,sim.num_Inputs,sim.num_Outputs,found);

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
                            else
                                out = homing(MAV_loop,depot,sim);
                                MAV_loop.u1 = out(1);
                                MAV_loop.u2 = out(2);
                            end
%                             state = behaviourTree(agent,depot,i,grid_MS,MS_temp);
% 
%                             switch state
%                                 case 1
%                                     MAV_loop.u1 = 0;
%                                     MAV_loop.u2 = 0;
%                                     MAV_loop.workBool = 1;
%                                     MAV_loop.chargeBool = 0;
%                                     MAV_loop.waitBool = 0;
%                                 case 2
%                                     out = homing(MAV_loop,depot,sim);
%                                     MAV_loop.u1 = out(1);
%                                     MAV_loop.u2 = out(2);
% 
%                                     MAV_loop.workBool = 0;
%                                     MAV_loop.chargeBool = 0;
%                                     MAV_loop.waitBool = 0;
%                                 case 3
%                                     MAV_loop.u1 = 0;
%                                     MAV_loop.u2 = 0;
% 
%                                     MAV_loop.workBool = 0;
%                                     MAV_loop.chargeBool = 0;
%                                     MAV_loop.waitBool = 1;
%                                 case 4
%                                     MAV_loop.u1 = 0;
%                                     MAV_loop.u2 = 0;
% 
%                                     MAV_loop.workBool = 0;
%                                     MAV_loop.chargeBool = 1;
%                                     MAV_loop.waitBool = 0;
%                                 case 5
%                                     MAV_loop.workBool = 1;
%                                     MAV_loop.chargeBool = 0;
%                                     MAV_loop.waitBool = 0;
% 
%                                     %--Calculate the output of the NN
%                                     out = calcNN_edit(test(j).pop(topController(contr,2)),hiddenLayers,sim.num_Inputs,sim.num_Outputs,found);
% 
%                                     %--Calculating the value of the velocity outputs
%                                     if out(1) ~= 0
%                                         theta = atan(abs(out(2)/out(1)));
%                                     else
%                                         theta = pi/2;
%                                     end
%                                     MAV_loop.u1 = out(1)*MAV_loop.maxVel*cos(theta);
%                                     MAV_loop.u2 = out(2)*MAV_loop.maxVel*sin(theta);
%                                     if sqrt(MAV_loop.u1^2+MAV_loop.u2^2) - MAV_loop.maxVel > 0.001
%                                         disp('MAX Vel Exceeded')
%                                     end
%                             end

                            %--Simulate the agents actions in the environment for the given
                            %Call agentDynamics to update position
                            MAV_loop = agentDynamics_D(MAV_loop, sim);
%                             MAV_loop = fuel(MAV_loop,sim);
                            agent(i) = MAV_loop;

                            sumCharge = sum([agent(:).chargeBool]);
                            if sumCharge > 0
                                depot.inUse = sumCharge;
                            else
                                depot.inUse = 0;
                            end
                         end %-- End of the case for inFormat 7
                    end%-- End of the if crashed statement
                end%--End of the number of agents for loop

                %Call ageMS to age the mission space
%                 [MS_temp,tickCount] = visited_CCode2(grid_MS,MS_temp,agent,sim,tickCount);
                [MS_temp,tickCount] = visited_D(grid_MS,MS_temp,agent,tickCount,sim);
                
                %--Update performance indices
                cntrl.aveAge(itt,floor(t/sim.ts+1)) = calcTotalAge_D(MS_temp)/grid_MS.numCells;
                cntrl.coveragePerc(itt,floor(t/sim.ts+1)) = (grid_MS.numCells-sum(tickCount(:) == 0)) / grid_MS.numCells*100;
                
                minDistance = 3;    %--This is the threshold for when the penalty is applied given in the article
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
                if minDistance <= 0.3
                    cntrl.crashed(itt,floor(t/sim.ts+1)) = 1;
                end
            
            end%-- end sim time FOR loop
            
       end%--End of itterations FOR loop
       aveAge(contr,:) = mean(cntrl.aveAge);
       ageSTD(contr,:) = std(cntrl.aveAge);
       coveragePerc(contr,:) = mean(cntrl.coveragePerc);
       crash(contr,:) = sum(cntrl.crashed');
       
%        %--Plotting performance
%        figure(j)
%        subplot(2,1,1)
%        plot(0:sim.ts:sim.time,aveAge(contr,:)) 
%        hold on
%        subplot(2,1,2)
%        plot(0:sim.ts:sim.time,coveragePerc(contr,:)) 
%        hold on

        
    end%--End of controller FOR loop
    reportAge(((j-1)*5+1):((j-1)*5+5),:) = aveAge;
    reportSTD(((j-1)*5+1):((j-1)*5+5),:) = ageSTD;
    reportCOV(((j-1)*5+1):((j-1)*5+5),:) = coveragePerc;
    reportCrash(((j-1)*5+1):((j-1)*5+5),:) = crash;
    
%     subplot(2,1,1)
%     xlabel('Time, [s]')
%     ylabel('Average cell age')
%     title('Cell Age Comparison')
%     legend('Controller 1','Controller 2','Controller 3','Controller 4','Controller 5')
%     drawnow
%     hold off
%     subplot(2,1,2)
%     xlabel('Time, [s]')
%     ylabel('Minimum Cell Ticks')
%     title('Coverage Percentage')
%     legend('Controller 1','Controller 2','Controller 3','Controller 4','Controller 5')
%     drawnow
%     hold off
    disp('-----------------------------------------')
    j
    
%     csvwrite(['1 aveAge ',num2str(j),'.csv'],aveAge');
%     csvwrite(['2 coveragePerc ',num2str(j),'.csv'],coveragePerc');
%     csvwrite(['1 aveAge ',num2str(j),'.csv'],[aveAge' ageSTD']);
%     csvwrite(['2 coveragePerc ',num2str(j),'.csv'],[coveragePerc' coverageSTD']);
    
end
    time = 0:sim.ts:sim.time;
    csvwrite(['input',num2str(sim.inFormat),'_noHighlevel_newDynSlow_neatsaveAge_',num2str(fuckThis),'_2525.csv'],[time',reportAge',reportSTD']);
    csvwrite(['input',num2str(sim.inFormat),'_noHighlevel_newDynSlow_neatsaveCov_',num2str(fuckThis),'_2525.csv'],[time',reportCOV']);
    csvwrite(['input',num2str(sim.inFormat),'_noHighlevel_newDynSlow_neatsaveCrash_',num2str(fuckThis),'_2525.csv'],[reportCrash']);
end

end

%% -- More tests

run_test3 = 0;
if run_test3==1

numCntrl = 3;

%-Load best results from durate test
% i=1;
% load('Results - Pure Duarte\neatsave - 11','population')
% test(i).pop = population;
% i=2;
% load('Results - Pure Duarte\neatsave - 11','population')
% test(i).pop = population;
% i=3;
% load('Results - Pure Duarte\neatsave - 11','population')
% test(i).pop = population;
% i=4;
% load('Results - Pure Duarte\neatsave - 12','population')
% test(i).pop = population;
% i=5;
% load('Results - Pure Duarte\neatsave - 12','population')
% test(i).pop = population;
% i=6;
% load('Results - Pure Duarte\neatsave - 12','population')
% test(i).pop = population;
% index = [8,9,10,2,4,15];


%--Load best results from input format 4
% i=1;
% fileName = 'Results - Alt inputs\neatsave - 2';
% load(fileName,'population')
% test(i).pop = population;
% i=2;
% fileName = 'Results - Alt inputs\neatsave - 2';
% load(fileName,'population')
% test(i).pop = population;
% i=3;
% fileName = 'Results - Alt inputs\neatsave - 3';
% load(fileName,'population')
% test(i).pop = population;
% i=4;
% fileName = 'Results - Alt inputs\neatsave - 3';
% load(fileName,'population')
% test(i).pop = population;
% index=[8,10,5,10];

%--Load best results from input format 5
% i=1;
% fileName = 'Results - Alt inputs\neatsave - 5';
% load(fileName,'population')
% test(i).pop = population;
% i=2;
% fileName = 'Results - Alt inputs\neatsave - 6';
% load(fileName,'population')
% test(i).pop = population;
% i=3;
% fileName = 'Results - Alt inputs\neatsave - 7';
% load(fileName,'population')
% test(i).pop = population;
% index=[13,3,7];

%--Load best results from input format 6
i=1;
fileName = 'Results - Alt inputs\neatsave - 10';
load(fileName,'population')
test(i).pop = population;
i=2;
fileName = 'Results - Alt inputs\neatsave - 11';
load(fileName,'population')
test(i).pop = population;
i=3;
fileName = 'Results - Alt inputs\neatsave - 11';
load(fileName,'population')
test(i).pop = population;
index=[5,13,5];

%--------- best_pop data
%==========================================================================
%--Load best results from input format 2
% i=1;
% fileName = 'Results - Pure Duarte\best_pop - 11';
% load(fileName,'population')
% test(i).pop = population;
% i=2;
% fileName = 'Results - Pure Duarte\best_pop - 11';
% load(fileName,'population')
% test(i).pop = population;
% i=3;
% fileName = 'Results - Pure Duarte\best_pop - 12';
% load(fileName,'population')
% test(i).pop = population;
% i=4;
% fileName = 'Results - Pure Duarte\best_pop - 12';
% load(fileName,'population')
% test(i).pop = population;
% i=5;
% fileName = 'Results - Pure Duarte\best_pop - 13';
% load(fileName,'population')
% test(i).pop = population;
% index=[7,8,15,111,7];

%--Load best results from input format 4
% i=1;
% fileName = 'Results - Alt inputs\best_pop - 1';
% load(fileName,'population')
% test(i).pop = population;
% i=2;
% fileName = 'Results - Alt inputs\best_pop - 2';
% load(fileName,'population')
% test(i).pop = population;
% i=3;
% fileName = 'Results - Alt inputs\best_pop - 3';
% load(fileName,'population')
% test(i).pop = population;
% index=[6,119,5];

% %--Load best results from input format 5
% i=1;
% fileName = 'Results - Alt inputs\best_pop - 5';
% load(fileName,'population')
% test(i).pop = population;
% i=2;
% fileName = 'Results - Alt inputs\best_pop - 5';
% load(fileName,'population')
% test(i).pop = population;
% i=3;
% fileName = 'Results - Alt inputs\best_pop - 5';
% load(fileName,'population')
% test(i).pop = population;
% i=4;
% fileName = 'Results - Alt inputs\best_pop - 6';
% load(fileName,'population')
% test(i).pop = population;
% i=5;
% fileName = 'Results - Alt inputs\best_pop - 7';
% load(fileName,'population')
% test(i).pop = population;
% i=6;
% fileName = 'Results - Alt inputs\best_pop - 7';
% load(fileName,'population')
% test(i).pop = population;
% i=7;
% fileName = 'Results - Alt inputs\best_pop - 7';
% load(fileName,'population')
% test(i).pop = population;
% index=[7,10,11,4,34,4,39];
% 
% %--Load best results from input format 6
% i=1;
% fileName = 'Results - Alt inputs\best_pop - 10';
% load(fileName,'population')
% test(i).pop = population;
% i=2;
% fileName = 'Results - Alt inputs\best_pop - 10';
% load(fileName,'population')
% test(i).pop = population;
% i=3;
% fileName = 'Results - Alt inputs\best_pop - 11';
% load(fileName,'population')
% test(i).pop = population;
% i=4;
% fileName = 'Results - Alt inputs\best_pop - 11';
% load(fileName,'population')
% test(i).pop = population;
% index=[28,2,126,127];

    %--re-initialise performace indices
    aveAge = zeros(numCntrl,sim.time/sim.ts+1);           %--Average cell age over time
    coverTime = zeros(numCntrl,1);                         %--Time until area is completely covered
    coverageCount = zeros(numCntrl,sim.time/sim.ts+1);    %--Number of times each cell is vivited over time
    zeroCount = zeros(numCntrl,sim.time/sim.ts+1);        %--Number of unvisited cells over time
    coveragePerc = zeros(numCntrl,sim.time/sim.ts+1);                      %--Percentage of area covered
    coverageSTD = zeros(numCntrl,1);                       %--Standard deviation of average coverage

for j=1:numCntrl
    figure(1)

        %--Performance indices for the different itterations of same
        %controller
        cntrl.aveAge = zeros(numItterations,sim.time/sim.ts+1);
        cntrl.coverTime = zeros(numItterations,1);
        cntrl.coverageCount = zeros(numItterations,sim.time/sim.ts+1);
        cntrl.zeroCount = zeros(5,sim.time/sim.ts+1);
        cntrl.coveragePerc = zeros(numItterations,sim.time/sim.ts+1);
        
        %--Create the layout of the NN
        [hiddenLayers,found] = formatNN_edited(test(j).pop(index(j)),sim.num_Inputs);
            
        %--Run the Simulation for n itterations
        for itt=1:numItterations
            [tickCount,X,Y] = initEnviron_D(grid_MS);
            tickCount = tickCount-50;
            tickCount=tickCount./51;

            %--Reinitialise the agents and the mission space
            MS_temp = MS;
            agent = MAV;
            
            %--Change starting Position so that for each itteration the
            %MAVs are slighlty further away from the center of the MS
            agent(1).posX = agent(1).posX;
            agent(1).posY = agent(1).posY + itt*ittOffset;
            agent(2).posX = agent(2).posX + itt*ittOffset*cosd(45);
            agent(2).posY = agent(2).posY + itt*ittOffset*sind(45);
            agent(3).posX = agent(3).posX+ itt*ittOffset;
            agent(3).posY = agent(3).posY;
            agent(4).posX = agent(4).posX + itt*ittOffset*cosd(45);
            agent(4).posY = agent(4).posY - itt*ittOffset*sind(45);
            agent(5).posX = agent(5).posX;
            agent(5).posY = agent(5).posY - itt*ittOffset;
            agent(6).posX = agent(6).posX - itt*ittOffset*cosd(45);
            agent(6).posY = agent(6).posY - itt*ittOffset*sind(45);
            agent(7).posX = agent(7).posX - itt*ittOffset;
            agent(7).posY = agent(7).posY;
            agent(8).posX = agent(8).posX - itt*ittOffset*cosd(45);
            agent(8).posY = agent(8).posY + itt*ittOffset*sind(45);

            for t=0:sim.ts:sim.time    % Run sim over the give time horizon
                for i=1:sim.numAgents

                    MAV_loop = agent(i);

                    if ~MAV_loop.crashed 
                        %--Assigne the inputs to the populations
                        test(j).pop(index(j)) = nn_inputs(MAV_loop,grid_MS,MS_temp,test(j).pop(index(j)),sim,agent,i,sim.inFormat);

                        %--Calculate the output of the NN
                        out = calcNN_edit(test(j).pop(index(j)),hiddenLayers,sim.num_Inputs,sim.num_Outputs,found);

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

                        %--Simulate the agents actions in the environment for the given
                        %Call agentDynamics to update position
                        MAV_loop = agentDynamics_D(MAV_loop, grid_MS);
                        agent(i) = MAV_loop;

                    end%-- End of the if crashed statement
                end%--End of the number of agents for loop

                %Call ageMS to age the mission space
                [MS_temp,tickCount] = visited_D(grid_MS,MS_temp,agent,tickCount);
                
                %--Update performance indices
                cntrl.aveAge(itt,t/sim.ts+1) = calcTotalAge_D(MS_temp)/grid_MS.numCells;
                if cntrl.coverTime(itt) == 0 && sum(tickCount(:) == 0) == 0
                    cntrl.coverTime(itt) = t;
                end
                
%                 [y,x] = find(MS(:) ~= -1);
                cntrl.coverageCount(itt,t/sim.ts+1) = min(tickCount(tickCount>=0));
                cntrl.coveragePerc(itt,t/sim.ts+1) = (grid_MS.numCells-sum(tickCount(:) == 0)) / grid_MS.numCells*100;
            end%-- end sim time FOR loop
%             cntrl.coveragePerc(itt) = (grid.numCells-sum(tickCount(:) == 0)) / grid.numCells*100;
            
       end%--End of itterations FOR loop
       
       aveAge(j,:) = mean(cntrl.aveAge);
       coverTime(j) = mean(cntrl.coverTime);
       coverageCount(j,:) = mean(cntrl.coverageCount);
       coveragePerc(j,:) = mean(cntrl.coveragePerc);
%        coverageSTD(j) = std(cntrl.coveragePerc);
       
%   
%        %--Plotting performance
%        figure(1)
% %        subplot(2,1,1)
%        plot(0:sim.ts:sim.time,aveAge(j,:),'k') 
%        hold on
% %        subplot(2,1,2)
% %        plot(0:sim.ts:sim.time,coverageCount(j,:),'b') 
% %        hold on

end
csvwrite('1 aveAge.csv',aveAge');
csvwrite('2 coveragePerc.csv',coveragePerc');
% csvwrite('coverageSTD.csv',coverageSTD');

    disp('-----------------------------------------')
    mean(aveAge')
    coverTime
    mean(coveragePerc')
end
