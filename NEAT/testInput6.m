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

clc
clear
close all

Font_size = 18;
set(0,'DefaultAxesXGrid','on','DefaultAxesYGrid','on',...
    'DefaultAxesZGrid','on');
set(0,'defaultFigurePosition',[488 100 560*1.6 420*1.4]);
set(0,'defaultFigurePaperPositionMode','auto');

%% --Initialise parameters
grid_MS2.width = 25; %40       % width of rectangular mission space in [m]
grid_MS2.bredth  = 25; %40     % bredth of rectangular mission space in [m] (Actually height)
grid_MS2.res = 1.0;           % resolution of the discretisation
grid_MS2.posTol = 0.1;      % positional tolerance of the agents
grid_MS2.numCells = 0;
[MS2,X,Y] = initEnviron_D(grid_MS2);
grid_MS2.numX = size(X,2);
grid_MS2.numY = size(Y,2);
grid_MS2.numCells = sum(MS2(:) >= 0); %This counts the number of cells with positive values (i.e: the cells without obstacles)

obs.width = 3;
obs.height = 3;

% ---------- Defining the simulation parameters
sim.numAgents = 8;
sim.inFormat = 6;
sim.num_Outputs = 2;
sim.time = 300;          % Total simulation time in [s] 
sim.ts = 0.25;
t_crash = 1550;
t_crash2 = 1550;
if sim.inFormat == 1
    sim.num_Inputs = 5;
elseif sim.inFormat == 2 || sim.inFormat == 3
    sim.num_Inputs = 9;
elseif sim.inFormat == 5
    sim.num_Inputs = 13;
elseif sim.inFormat == 5 || sim.inFormat == 6 || sim.inFormat == 8
    sim.num_Inputs = 17;
elseif sim.inFormat == 7
    sim.num_Inputs = 2;
end   

depot.posX = 12.5;
depot.posY = 12.5;
depot.inUse = 0;
depot.charge = 0;

for i=1:sim.numAgents
    MAV(i).posX = grid_MS2.width/2+0.5*grid_MS2.res;        % [m]
    MAV(i).posY = grid_MS2.bredth/2+0.5*grid_MS2.res;       % [m]
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

    MAV(i).refuelRate = 5;%5,1.75         % Rate at which the UAV refuels itself while charging, [/s]
    MAV(i).workBool = 1;
    MAV(i).chargeBool = 0;          % Boolean, is MAV recharging
    MAV(i).waitBool = 0;
    MAV(i).fuel = 100;              % This represents the fuel level with 100 being fully charged and 0 as empty
    MAV(i).dischargeRate = 0.5;  %0.5, 0.208     % Rate at which fuel is consumed, [unit/s] 
    MAV(i).crashed = 0;             % Boolean stating whether the MAV has crashed
    MAV(i).avoiding = 0;
    MAV(i).state = 5;
end

%% Use this to run tests on the top five controllers from the given files
% numItterations = 50;
% fileNumbers = [1,2,4,5,6,7,8,9,11,12,13,14,15,16,17,18,28,29,30]; %1,2,4,5,6,7,8,9,11,12,13,14,15,16,17,18,28,29,30;  17,18,19,20,26,27
% numLoad = size(fileNumbers,2);
% %--Reading and storing the populations from the test files
% for i=1:numLoad
% %     fileName = ['Results - Alt inputs\best_pop - ',num2str(startIndx+i)];
% %     fileName = ['Results - Alt inputs\neatsave - ',num2str(fileNumbers(i))]; 
% %     fileName = ['Results - Final Inputs\neatsave - ',num2str(fileNumbers(i))];
%     fileName = ['Results - Final Inputs\neatsave - ',num2str(fileNumbers(i))];
%     load(fileName,'population')
%     
%     test(i).pop = population;
% end
% 
% 
% for j=1:numLoad
%     j
%     %--Finding the 5 controllers with the best fitness from current test
%     topController = zeros(5,2);
%     for i=1:size(test(j).pop,2)
%         [minVal,index] = min(topController(:,1));
%         if test(j).pop(i).fitness > minVal
%             topController(index,:) = [test(j).pop(i).fitness i];
%         end
%     end
% 
%     for contr=1:5
%         contr
%         %--Performance indices for the different itterations of same
%         %controller
%         cntrl.aveAge = zeros(numItterations,sim.time/sim.ts+1);
%         cntrl.coveragePerc = zeros(numItterations,sim.time/sim.ts+1);
%         cntrl.crashed = zeros(numItterations,sim.time/sim.ts+1);
% 
%         %--Create the layout of the NN
%         [hiddenLayers,found] = formatNN_edited(test(j).pop(topController(contr,2)),sim.num_Inputs);
% 
%         %--Run the Simulation for n itterations
%         for itt=1:numItterations           
%             grid_loop = grid_MS2;
% %             grid_loop.width = randi([floor(grid_MS2.width*0.75) grid_MS2.width],1,1);
% %             grid_loop.bredth  = randi([floor(grid_MS2.bredth*0.75) grid_MS2.bredth],1,1);
% %             [MS,X,Y] = initEnvirons_2(grid_loop, obs);
%             [MS,X,Y] = initEnviron_D(grid_loop);
%             grid_loop.numX = size(X,2);
%             grid_loop.numY = size(Y,2);
%             grid_loop.numCells = sum(MS(:) >= 0);
% 
%             tickCount = MS;
% 
%             %--Reinitialise the agents and the mission space
%             MS_temp = MS;
%             agent = MAV;
%             grid_MS = grid_loop;
%             for i=1:sim.numAgents
%                 inObs = 1;
%                 while inObs
%                     temp_indxX = grid_MS.res + (grid_MS.width-grid_MS.res*2)*rand(1,1);
%                     temp_indxY = grid_MS.res + (grid_MS.bredth-grid_MS.res*2)*rand(1,1);
%                     if MS_temp(ceil(temp_indxY/grid_MS.res),ceil(temp_indxX/grid_MS.res)) ~= -1
%                         inObs = 0;
%                         agent(i).posX = temp_indxX;
%                         agent(i).posY = temp_indxY;
%                     end
%                 end
%             end
%             %--Initialise the depot 
%             depot.posX = grid_loop.width /2;
%             depot.posY = grid_loop.bredth/2;
%             depot.inUse = 0;
% 
%             for t=0:sim.ts:sim.time    % Run sim over the give time horizon
%                 for i=1:sim.numAgents
%                     MAV_loop = agent(i);
% 
%                     if ~MAV_loop.crashed 
%                         %--Assigne the inputs to the populations
%                         test(j).pop(topController(contr,2)) = nn_inputs(MAV_loop,grid_MS,MS_temp,test(j).pop(topController(contr,2)),sim,agent,i);
% 
%                         %--Calculate the output of the NN
%                         out = calcNN_edit(test(j).pop(topController(contr,2)),hiddenLayers,sim.num_Inputs,sim.num_Outputs,found);
% 
%                         %--Calculating the value of the velocity outputs
%                         if out(1) ~= 0
%                             theta = atan(abs(out(2)/out(1)));
%                         else
%                             theta = pi/2;
%                         end
%                         MAV_loop.u1 =  out(1)*MAV_loop.maxVel*cos(theta);
%                         MAV_loop.u2 =  out(2)*MAV_loop.maxVel*sin(theta);
%                         if sqrt(MAV_loop.u1^2+MAV_loop.u2^2) - MAV_loop.maxVel > 0.001
%                             disp('MAX Vel Exceeded')
%                         end
% 
%                         %--Simulate the agents actions in the environment for the given
%                         %Call agentDynamics to update position
%                         MAV_loop = agentDynamics_D(MAV_loop, sim);
%                         agent(i) = MAV_loop;
% 
% %                         test(j).pop(topController(contr,2)) = nn_inputs(MAV_loop,grid_MS,MS_temp,test(j).pop(topController(contr,2)),sim,agent,i);
% %                         state = behaviourTree(agent,depot,i,grid_MS,MS_temp);
% %                         switch state
% %                             case 1
% %                                 MAV_loop.u1 = 0;
% %                                 MAV_loop.u2 = 0;
% %                                 MAV_loop.workBool = 1;
% %                                 MAV_loop.chargeBool = 0;
% %                                 MAV_loop.waitBool = 0;
% %                             case 2
% %                                 out = homing(MAV_loop,depot,sim);
% %                                 MAV_loop.u1 = out(1);
% %                                 MAV_loop.u2 = out(2);
% % 
% %                                 MAV_loop.workBool = 0;
% %                                 MAV_loop.chargeBool = 0;
% %                                 MAV_loop.waitBool = 0;
% %                             case 3
% %                                 MAV_loop.u1 = 0;
% %                                 MAV_loop.u2 = 0;
% % 
% %                                 MAV_loop.workBool = 0;
% %                                 MAV_loop.chargeBool = 0;
% %                                 MAV_loop.waitBool = 1;
% %                             case 4
% %                                 MAV_loop.u1 = 0;
% %                                 MAV_loop.u2 = 0;
% % 
% %                                 MAV_loop.workBool = 0;
% %                                 MAV_loop.chargeBool = 1;
% %                                 MAV_loop.waitBool = 0;
% %                             case 5
% %                                 MAV_loop.workBool = 1;
% %                                 MAV_loop.chargeBool = 0;
% %                                 MAV_loop.waitBool = 0;
% % 
% %                                 %--Calculate the output of the NN
% %                                 out = calcNN_edit(test(j).pop(topController(contr,2)),hiddenLayers,sim.num_Inputs,sim.num_Outputs,found);
% % 
% %                                 %--Calculating the value of the velocity outputs
% %                                 if out(1) ~= 0
% %                                     theta = atan(abs(out(2)/out(1)));
% %                                 else
% %                                     theta = pi/2;
% %                                 end
% %                                 MAV_loop.u1 =  out(1)*MAV_loop.maxVel*cos(theta);
% %                                 MAV_loop.u2 =  out(2)*MAV_loop.maxVel*sin(theta);
% %                                 if sqrt(MAV_loop.u1^2+MAV_loop.u2^2) - MAV_loop.maxVel > 0.001
% %                                     disp('MAX Vel Exceeded')
% %                                 end
% %                         end
% % 
% %                         %--Simulate the agents actions in the environment for the given
% %                         %Call agentDynamics to update position
% %                         MAV_loop = agentDynamics_D(MAV_loop, sim);
% %                         MAV_loop = fuel(MAV_loop,sim);
% %                         agent(i) = MAV_loop;
% % 
% %                         sumCharge = sum([agent(:).chargeBool]);
% %                         if sumCharge > 0
% %                             depot.inUse = sumCharge;
% %                         else
% %                             depot.inUse = 0;
% %                         end
%                     end%-- End of the if crashed statement
%                 end%--End of the number of agents for loop
% 
%                 %Call ageMS to age the mission space
%                 [MS_temp,tickCount] = visited_D(grid_MS,MS_temp,agent,tickCount,sim);
% 
%                 %--Update performance indices
%                 cntrl.aveAge(itt,floor(t/sim.ts+1)) = calcTotalAge_D(MS_temp)/grid_MS.numCells;
%                 cntrl.coveragePerc(itt,floor(t/sim.ts+1)) = (grid_MS.numCells-sum(tickCount(:) == 0)) / grid_MS.numCells*100;
%                 
%                 minDistance = 3;    %--This is the threshold for when the penalty is applied given in the article
%                 for i=1:sim.numAgents   %--This finds the minimum distance between any two agents
%                     for agentCount=1:sim.numAgents
%                         if i ~= agentCount
%                             distance = sqrt((agent(agentCount).posX-agent(i).posX)^2+(agent(agentCount).posY-agent(i).posY)^2);
%                             if distance < minDistance
%                                 minDistance = distance;
%                             end
%                         end
%                     end   
%                 end
%                 if minDistance <= 0.3
%                     cntrl.crashed(itt,floor(t/sim.ts+1)) = 1;
%                 end
%             end%-- end sim time FOR loop
%        end%--End of itterations FOR loop
%        time = 0:sim.ts:sim.time;
% %        csvwrite(['cyberzooData2_2uavs_file',num2str(fileNumbers(j)),'_cntrl',num2str(topController(contr,2)),'_Age.csv'],[time', cntrl.aveAge']);
% %        csvwrite(['cyberzooData2_2uavs_file',num2str(fileNumbers(j)),'_cntrl',num2str(topController(contr,2)),'_Cov.csv'],[time',cntrl.coveragePerc']);
% %        csvwrite(['cyberzooData2_2uavs_file',num2str(fileNumbers(j)),'_cntrl',num2str(topController(contr,2)),'_Crash.csv'],[time',cntrl.crashed']);
% %        csvwrite(['2_RefuelFile',num2str(fileNumbers(j)),'_cntrl',num2str(topController(contr,2)),'_Age.csv'],[time', cntrl.aveAge']);
% %        csvwrite(['2_RefuelFile',num2str(fileNumbers(j)),'_cntrl',num2str(topController(contr,2)),'_Cov.csv'],[time',cntrl.coveragePerc']);
% %        csvwrite(['2_RefuelFile',num2str(fileNumbers(j)),'_cntrl',num2str(topController(contr,2)),'_Crash.csv'],[time',cntrl.crashed']);
%        csvwrite(['In2_newDynamicsSlow_file',num2str(fileNumbers(j)),'_cntrl',num2str(topController(contr,2)),'_Age.csv'],[time', cntrl.aveAge']);
%        csvwrite(['In2_newDynamicsSlow_file',num2str(fileNumbers(j)),'_cntrl',num2str(topController(contr,2)),'_Cov.csv'],[time',cntrl.coveragePerc']);
%        csvwrite(['In2_newDynamicsSlow_file',num2str(fileNumbers(j)),'_cntrl',num2str(topController(contr,2)),'_Crash.csv'],[time',cntrl.crashed']);
%     end%--End of controller FOR loop
% end
% 
% figure()

%-----------------------------------------------------------------------------------------------
%% This runs the tests for the top 15 controllers identified
%-- Top 20 controller data
numItterations = 1;

%-Cyber Zoo tests
% fileNumbers = [1,2,4,5,6,7,8,9,11,12,13,14,15,16,17,18,28,29,30];
% cntrlNum = [13,16,17,99,100,2,4,5,15,44,3,4,8,13,48,3,4,6,8,74,1,9,17,24,80,1,3,13,30,112,3,4,5,26,34,1,8,12,21,86,4,5,10,53,75,1,2,5,6,14,1,2,5,6,14,8,9,23,24,88,1,2,10,28,29,4,5,6,7,85,4,6,8,55,85,2,3,5,38,57,2,3,9,23,72,5,6,10,34,44,3,22,36,52,69];
% best = [36,37,38,40,41,42,44,60,66,71];

%--Standard tests
fileNumbers = [1,2,4,5,6,7,8,9,11,15,16,17,18,30];
cntrlNum = [13,16,17,99,100,2,4,5,15,44,3,4,8,13,48,3,4,6,8,74,1,9,17,24,80,1,3,13,30,112,3,4,5,26,34,1,8,12,21,86,4,5,10,53,75,1,2,10,28,29,4,5,6,7,85,4,6,8,55,85,2,3,5,38,57,3,22,36,52,69];
% best = [12,36,38,42,44,51,63,27,39]; %48,15,11
best=[1,6,15,33,36,38,42,44,51,63,27,39];%1,2,3,4,5,6,7,8,9,15,33,12,36,38,42,44,51,63,27,39

for i=1:12
    file = ceil(best(i)/5);
    fileName = ['Results - Final Inputs\neatsave - ',num2str(fileNumbers(file))];
    load(fileName,'population')
    test(i).pop = population;
    pop_index(i) = cntrlNum(best(i));
end

% fileName = ['Results - Alt inputs\neatsave - ',num2str(19)];
% load(fileName,'population')
% test(10).pop = population;

%--Cyber zoo tests
% cntrlNum = [2,10,11,15,87,5,10,44,45,80,2,6,11,149,150,1,7,8,22,60,5,8,17,42,85,1,5,11,19,83];
% fileNumbers = [17,18,19,20,26,27];
% best = [5,10,16,17,21];%1,2,3,5,6,7,8,9,12,13,17,13

%--Standard tests
cntrlNum = [2,10,11,15,87,5,10,44,45,80,2,6,11,149,150,1,7,8,22,60,5,8,17,42,85,1,5,11,19,83];
fileNumbers = [17,18,19,20,26,27];
% best = 1:30;%5,6,8,12,13,14,16,20,23,25
best = [3,5,6,8,9,12,13,17];%1,2,3,5,6,7,8,9,12,13,17,13

for i=1:8
    file = ceil(best(i)/5);
    fileName = ['Results - Alt Inputs\neatsave - ',num2str(fileNumbers(file))];
    load(fileName,'population')
    test(i+12).pop = population;
    pop_index(i+12) = cntrlNum(best(i));
end

% best=[36,37,38,40,41,42,44,60,66,71,5,10,16,17,21];
for j=8:8
    %file = ceil(best(j)/5);
    j
    %--Performance indices for the different itterations of same
    %controller
    cntrl.aveAge = zeros(numItterations,floor(sim.time/sim.ts)+1);
    cntrl.coveragePerc = zeros(numItterations,floor(sim.time/sim.ts)+1);
    cntrl.crashed = zeros(numItterations,floor(sim.time/sim.ts)+1);

    %--Create the layout of the NN
    [hiddenLayers,found] = formatNN_edited(test(j).pop(pop_index(i)),sim.num_Inputs);

    %--Run the Simulation for n itterations
    for itt=1:numItterations 
        itt
        grid_loop = grid_MS2;
%         grid_loop.width = randi([floor(grid_MS2.width*0.75) grid_MS2.width],1,1);
%         grid_loop.bredth  = randi([floor(grid_MS2.bredth*0.75) grid_MS2.bredth],1,1);

%         [MS,X,Y] = initEnvirons_2(grid_loop, obs);
        [MS,X,Y] = initEnviron_D(grid_loop);
        grid_loop.numX = size(X,2);
        grid_loop.numY = size(Y,2);
        grid_loop.numCells = sum(MS(:) >= 0);

        tickCount = MS;

        %--Reinitialise the agents and the mission space
        MS_temp = MS;
%         sim.numAgents = 8;
        agent = MAV;
        grid_MS = grid_loop;
        for i=1:sim.numAgents
            inObs = 1;
            while inObs
                temp_indxX = grid_MS.res + (grid_MS.width-grid_MS.res*2)*rand(1,1);
                temp_indxY = grid_MS.res + (grid_MS.bredth-grid_MS.res*2)*rand(1,1);
                if MS_temp(ceil(temp_indxY/grid_MS.res),ceil(temp_indxX/grid_MS.res)) ~= -1
                    inObs = 0;
                    agent(i).posX = temp_indxX;
                    agent(i).posY = temp_indxY;
%                     if i == 1
%                         inObs = 0;
%                         agent(i).posX = temp_indxX;
%                         agent(i).posY = temp_indxY;
%                     else
%                         if sqrt((agent(1).posX-agent(2).posX)^2+(agent(1).posY-agent(2).posY)^2) > 1
%                             inObs = 0;
%                             agent(i).posX = temp_indxX;
%                             agent(i).posY = temp_indxY;
%                         end
%                     end
                end
            end
        end
        
        %--Initialise the depot 
%         depot.posX = grid_loop.width /2;
%         depot.posY = grid_loop.bredth/2;
        depot.inUse = 0;
        depot.charge = 0;
        
        for t=0:sim.ts:sim.time    % Run sim over the give time horizon
            for i=1:sim.numAgents
                MAV_loop = agent(i);

                if ~MAV_loop.crashed 
%                     dist = sqrt((agent(1).posX-agent(2).posX)^2+(agent(1).posY-agent(2).posY)^2); %Between 1 and 2
%                     if dist > 0.8 && MAV_loop.avoiding == 0
%                         
%                         %--Assigne the inputs to the populations
%                         test(j).pop(pop_index(i)) = nn_inputs(MAV_loop,grid_MS,MS_temp,test(j).pop(pop_index(i)),sim,agent,i);
% 
%                         %--Calculate the output of the NN
%                         out = calcNN_edit(test(j).pop(pop_index(i)),hiddenLayers,sim.num_Inputs,sim.num_Outputs,found);
% 
%                         %--Calculating the value of the velocity outputs
%                         if out(1) ~= 0
%                             theta = atan(abs(out(2)/out(1)));
%                         else
%                             theta = pi/2;
%                         end
%                         MAV_loop.u1 =  out(1)*MAV_loop.maxVel*cos(theta);
%                         MAV_loop.u2 =  out(2)*MAV_loop.maxVel*sin(theta);
%                         if sqrt(MAV_loop.u1^2+MAV_loop.u2^2) - MAV_loop.maxVel > 0.001
%                             disp('MAX Vel Exceeded')
%                         end
%                     elseif dist > 1.0
%                         MAV_loop.avoiding = 0;
%                     else
%                         disp(['Before ',num2str(i),': ',num2str(MAV_loop.u1),', ',num2str(MAV_loop.u2)])
%                         [out,MAV_loop] = avoidance(MAV_loop,sim,grid_MS,depot);
%                         MAV_loop.u1 = out(1);
%                         MAV_loop.u2 = out(2);
%                         disp(['After ',num2str(i),': ',num2str(MAV_loop.u1),', ',num2str(MAV_loop.u2)])
%                     end

%                     %--Assigne the inputs to the populations
%                     test(j).pop(pop_index(i)) = nn_inputs(MAV_loop,grid_MS,MS_temp,test(j).pop(pop_index(i)),sim,agent,i);
% 
%                     %--Calculate the output of the NN
%                     out = calcNN_edit(test(j).pop(pop_index(i)),hiddenLayers,sim.num_Inputs,sim.num_Outputs,found);
% 
%                     %--Calculating the value of the velocity outputs
%                     if out(1) ~= 0
%                         theta = atan(abs(out(2)/out(1)));
%                     else
%                         theta = pi/2;
%                     end
%                     MAV_loop.u1 =  out(1)*MAV_loop.maxVel*cos(theta);
%                     MAV_loop.u2 =  out(2)*MAV_loop.maxVel*sin(theta);
%                     if sqrt(MAV_loop.u1^2+MAV_loop.u2^2) - MAV_loop.maxVel > 0.001
%                         disp('MAX Vel Exceeded')
%                     end
% 
%                     %--Simulate the agents actions in the environment for the given
%                     %Call agentDynamics to update position
%                     MAV_loop = agentDynamics_D(MAV_loop, sim);
%                     agent(i) = MAV_loop;

                    test(j).pop(pop_index(i)) = nn_inputs(MAV_loop,grid_MS,MS_temp,test(j).pop(pop_index(i)),sim,agent,i);
%                     state = behaviourTree(agent,depot,i,grid_MS,sim);
%                     MAV_loop.state = state;
%                     switch state
%                         case 1
%                             [out,MAV_loop] = avoidance(MAV_loop,sim,grid_MS,depot);
%                             MAV_loop.u1 = out(1);
%                             MAV_loop.u2 = out(2);
%                             MAV_loop.workBool = 1;
%                             MAV_loop.chargeBool = 0;
%                             MAV_loop.waitBool = 0;
%                         case 2
%                             out = homing(MAV_loop,depot,sim);
%                             MAV_loop.u1 = out(1);
%                             MAV_loop.u2 = out(2);
% 
%                             MAV_loop.avoiding = 0;
%                             MAV_loop.workBool = 0;
%                             MAV_loop.chargeBool = 0;
%                             MAV_loop.waitBool = 0;
%                         case 3
%                             MAV_loop.u1 = 0;
%                             MAV_loop.u2 = 0;
% 
%                             MAV_loop.avoiding = 0;
%                             MAV_loop.workBool = 0;
%                             MAV_loop.chargeBool = 0;
%                             MAV_loop.waitBool = 1;
% 
%                             if sum([agent(:).state] == 4) == 0
%                                 MAV_loop.state = 4;
%                                 MAV_loop.chargeBool = 1;
%                                 MAV_loop.waitBool = 0;
%                             end
%                         case 4
%                             MAV_loop.u1 = 0;
%                             MAV_loop.u2 = 0;
% 
%                             MAV_loop.avoiding = 0;
%                             MAV_loop.workBool = 0;
%                             MAV_loop.chargeBool = 1;
%                             MAV_loop.waitBool = 0;
%                         case 5
%                             MAV_loop.workBool = 1;
%                             MAV_loop.chargeBool = 0;
%                             MAV_loop.waitBool = 0;
%                             MAV_loop.avoiding = 0;

                            %--Calculate the output of the NN
                            out = calcNN_edit(test(j).pop(pop_index(i)),hiddenLayers,sim.num_Inputs,sim.num_Outputs,found);

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
%                     end
                    %--Simulate the agents actions in the environment for the given
                    %Call agentDynamics to update position
                    MAV_loop = agentDynamics_D(MAV_loop, sim);
%                     MAV_loop = fuel(MAV_loop,sim);
                    agent(i) = MAV_loop;

                    depot.charge = sum([agent(:).state] == 3)+ (sum([agent(:).state] == 4));
                    depot.inUse = sim.numAgents-sum([agent(:).workBool]);
                        
                end%-- End of the if crashed statement
            end%--End of the number of agents for loop

            %Call ageMS to age the mission space
            [MS_temp,tickCount] = visited_D(grid_MS,MS_temp,agent,tickCount,sim);

            %--Update performance indices
            cntrl.aveAge(itt,floor(t/sim.ts+1)) = calcTotalAge_D(MS_temp)/grid_MS.numCells;
            cntrl.coveragePerc(itt,floor(t/sim.ts+1)) = (grid_MS.numCells-sum(tickCount(:) == 0)) / grid_MS.numCells*100;

            minDistance = 3;    %--This is the threshold for when the penalty is applied given in the article
            for i=1:sim.numAgents   %--This finds the minimum distance between any two agents
                if agent(i).state == 5 || agent(i).state == 1
                    for agentCount=1:sim.numAgents
                        if i ~= agentCount && (agent(agentCount).state == 5 || agent(agentCount).state == 1)
                            distance = sqrt((agent(agentCount).posX-agent(i).posX)^2+(agent(agentCount).posY-agent(i).posY)^2);
                            if distance < minDistance
                                minDistance = distance;
                            end
                        end
                    end  
                end
            end          
            if minDistance <= 0.3
                cntrl.crashed(itt,floor(t/sim.ts+1)) = 1;
            end
            
%             %--Crash agent 8 at time t_crash
            if t==t_crash
                sim.numAgents = 7;
                agent = agent(1:7);
            end
            if t==t_crash2
                sim.numAgents = 6;
                agent = agent(1:6); 
            end
            tempFuel(t/sim.ts+1,:) = [agent(1).fuel agent(2).fuel agent(3).fuel agent(4).fuel agent(5).fuel agent(6).fuel];
        end%-- end sim time FOR loop
   end%--End of itterations FOR loop
   time = 0:sim.ts:sim.time;
%    csvwrite(['baselineComp_6_file',num2str(fileNumbers(file)),'_cntrl',num2str(cntrlNum(best(j))),'_Age.csv'],[time', cntrl.aveAge']);
%    csvwrite(['baselineComp_6_file',num2str(fileNumbers(file)),'_cntrl',num2str(cntrlNum(best(j))),'_Cov.csv'],[time',cntrl.coveragePerc']);
%    csvwrite(['baselineComp_6_file',num2str(fileNumbers(file)),'_cntrl',num2str(cntrlNum(best(j))),'_Crash.csv'],[time',cntrl.crashed']);
%    csvwrite(['robust2_newDynamicsSlow_file',num2str(j),'_Age.csv'],[time', cntrl.aveAge']);
%    csvwrite(['robust2_newDynamicsSlow_file',num2str(j),'_Cov.csv'],[time',cntrl.coveragePerc']);
%    csvwrite(['robust2_newDynamicsSlow_file',num2str(j),'_Crash.csv'],[time',cntrl.crashed']);
%    csvwrite(['scaleData2_newDynamics_slow_file_',num2str(j),'_Age.csv'],[time', cntrl.aveAge']);
%    csvwrite(['scaleData2_newDynamics_slow_file_',num2str(j),'_Cov.csv'],[time',cntrl.coveragePerc']);
%    csvwrite(['scaleData2_newDynamics_slow_file_',num2str(j),'_Crash.csv'],[time',cntrl.crashed']);
%    csvwrite(['cyberzoo_2uavs_newDynamic_czCntrl_file_',num2str(j),'_Age.csv'],[time', cntrl.aveAge']);
%    csvwrite(['cyberzoo_2uavs_newDynamic_czCntrl_file_',num2str(j),'_Cov.csv'],[time',cntrl.coveragePerc']);
%    csvwrite(['cyberzoo_2uavs_newDynamic_czCntrl_file_',num2str(j),'_Crash.csv'],[time',cntrl.crashed']);
%    csvwrite(['avoidance1_newDynamicsSlow_file_',num2str(j),'_Age.csv'],[time', cntrl.aveAge']);
%    csvwrite(['avoidance1_newDynamicsSlow_file_',num2str(j),'_Cov.csv'],[time',cntrl.coveragePerc']);
%    csvwrite(['avoidance1_newDynamicsSlow_file_',num2str(j),'_Crash.csv'],[time',cntrl.crashed']);
%    csvwrite(['refuelData3_long_newDynamicsSlow_file_',num2str(j),'_Age.csv'],[time', cntrl.aveAge']);
%    csvwrite(['refuelData3_long_newDynamicsSlow_file_',num2str(j),'_Cov.csv'],[time',cntrl.coveragePerc']);
%    csvwrite(['refuelData3_long_newDynamicsSlow_file_',num2str(j),'_Crash.csv'],[time',cntrl.crashed']);
%        csvwrite(['newDynamics_slow_file_',num2str(j),'_Age.csv'],[time', cntrl.aveAge']);
%        csvwrite(['newDynamics_slow_file_',num2str(j),'_Cov.csv'],[time',cntrl.coveragePerc']);
%        csvwrite(['newDynamics_slow_file_',num2str(j),'_Crash.csv'],[time',cntrl.crashed']);

% time = 0:sim.ts:sim.time;
% figure()
% plot(time,tempFuel(:,1),'k','linewidth',1)
% hold on
% plot(time,tempFuel(:,2),'c','linewidth',1)
% plot(time,tempFuel(:,3),'r','linewidth',1)
% plot(time,tempFuel(:,4),'b','linewidth',1)
% plot(time,tempFuel(:,5),'g','linewidth',1)
% plot(time,tempFuel(:,6),'y','linewidth',1)

        %--Plot the tickCounters
        figure()
        surface(X,Y,tickCount)
        colorbar
        title('Number of cell visits')
        xlabel('Mission space length')
        ylabel('Mission space width')
        set(gca,'fontsize',Font_size)
        
end%--End of controller FOR loop

figure()