%% MSc Thesis:
% Thomas Fijen, 4620852

%% ------------ runRandomWalk
%   This script implements the basic random walk surveillance
%   Date: 23 Feb 2018
%   UPDATE: 1) extended to the multi agent case

clc
clear
close all

Font_size = 18;

set(0,'DefaultAxesXGrid','on','DefaultAxesYGrid','on',...
    'DefaultAxesZGrid','on');
set(0,'defaultFigurePosition',[488 100 560*1.6 420*1.4]);
set(0,'defaultFigurePaperPositionMode','auto');

%% ---------- TEST: Multi agent Random walk
itterations = 1;
ittOffset = 1;

grid_MS.width = 25;             % width of rectangular mission space in [m]
grid_MS.bredth  = 25;           % breadth of rectangular mission space in [m] (Actually height)
grid_MS.res = 1;                % resolution of the discretisation
grid_MS.posTol = 0.1;           % positional tolerance of the agents
grid_MS.numCells = 0;           % Number of non obstacle cells
t_crash = 10000;                % Time at which first agent crashes
t_crash2 = 10000;               % Time at which 2nd agent crashes
[MS,X,Y] = initEnviron_B(grid_MS);
grid_MS.numX = size(X,2);
grid_MS.numY = size(Y,2);
grid_MS.numCells = sum(MS(:) >= 0); %This counts the number of cells with positive values (i.e: the cells without obstacles)

%-- Plotting the age map
% figure()
% mesh(X,Y,MS)
% title('Age Map');
% colorbar;

% ---------- Defining the simulation parameters
sim.numAgents = 8;
sim.time = 300;             % Total simulation time in [s] 
sim.ts = 0.25;               % Size of the time steps in [s]

%--Initialise the depot 
depot.posX = 12.5;
depot.posY = 12.5;
depot.inUse = 0;
depot.charge = 0;
    
% ---------- Defining the agents
for i=sim.numAgents:-1:1
    MAV(i).posX = 10;               % [m]
    MAV(i).posY = 10;               % [m]
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
    MAV(i).targetFlag = 0;          % 1 if moving to assigned target cells
    MAV(i).u1 = 1;                  %This is the first output of the NN. Corresponds to: Velocity X
    MAV(i).u2 = 0;                  % This is the second output of the NN. Coresponds to: Velocity Y
    MAV(i).sensorRange = 10;        % Sensor range in [m]
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
    MAV(i).state = 6;
end

aveAge = zeros(itterations,sim.time/sim.ts+1);           %--Average cell age over time
coverTime = zeros(itterations,1);                        %--Time until area is completely covered
coverageCount = zeros(itterations,sim.time/sim.ts+1);    %--Number of times each cell is vivited over time
coveragePerc = zeros(itterations,sim.time/sim.ts+1);     %--Percentage of area covered
crashed = zeros(itterations,sim.time/sim.ts+1); 

% MAV_loop = MAV(1);
% figure
% plot(MAV_loop.posX,MAV_loop.posY,'o')
% hold on
% for i=1:20
%     MAV_loop = agentDynamics_B(MAV_loop, sim);
%     [out,MAV_loop] = avoidance(MAV_loop,sim,grid_MS,depot);
%     MAV_loop.u1 = out(1);
%     MAV_loop.u2 = out(2);
%     plot(MAV_loop.posX,MAV_loop.posY,'o')
% end
% out

%% ---------- updating the positions
targets = zeros(sim.numAgents,2);
interUAVdist = zeros(sim.time/sim.ts,1);
for itt = 1:itterations
    itt
%     sim.numAgents = 8;
    [tickCount,X,Y] = initEnviron_B(grid_MS);
    tickCount = tickCount-99.75;
    tickCount = tickCount/100.75;
    %--Reinitialise the agents and the mission space
    MS_temp = MS;
    agent = MAV;
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

% figure()
% plot(MAV(1).posX,MAV(1).posY,'bo')
% hold on
% plot(MAV(2).posX,MAV(2).posY,'ro')
% plot(MAV(3).posX,MAV(3).posY,'ko')
% plot(MAV(4).posX,MAV(4).posY,'go')
% plot(MAV(5).posX,MAV(5).posY,'mo')
% plot(MAV(6).posX,MAV(6).posY,'o','Color',[160/255 32/255 240/255])
% plot(MAV(7).posX,MAV(7).posY,'c+')
% plot(MAV(8).posX,MAV(8).posY,'+','Color',[160/255 100/255 150/255])
% xlabel('Mission space length');
% ylabel('Mission space width')
% legend('MAV 1','MAV 2')
% title('Avoidance Maneuver')
% set(gca,'fontsize',Font_size)

%--Data for displaying avoidance movement
% agent(1).posX = 5;
% agent(1).posY = 2;
% agent(2).posX = 5;
% agent(2).posY = 9;
% counter1 = 1;
% counter2 = 1;
% targetPositions = [5 9 5 2;12 2 12 9];
% recPos = zeros(100,4);
    
    for t=0:sim.ts:sim.time
        for i=1:sim.numAgents
            MAV_loop = agent(i);
            if ~MAV_loop.crashed
                state = behaviourTree(agent,depot,i,grid_MS,sim);
%                 disp(['state',num2str(state)])
%                 state = 5;
                MAV_loop.state = state;
                switch state
                    case 1
                        [out,MAV_loop] = avoidance(MAV_loop,sim,grid_MS,depot);
                        MAV_loop.u1 = out(1);
                        MAV_loop.u2 = out(2);
                        MAV_loop.workBool = 1;
                        MAV_loop.chargeBool = 0;
                        MAV_loop.waitBool = 0;
%                         disp(['Agent ',num2str(i),' Avoiding'])
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

                        if agent(i).targetFlag == 0
                            MAV_loop = randomWalk(MS_temp,grid_MS,MAV_loop,targets);
                            targets(i,:) = [MAV_loop.targetX MAV_loop.targetY];
%                             if i == 1
%                                 MAV_loop.targetX = targetPositions(counter1,1);
%                                 MAV_loop.targetY = targetPositions(counter1,2);
%                                 counter1 = counter1+1;
%                             else
%                                 MAV_loop.targetX = targetPositions(counter2,3);
%                                 MAV_loop.targetY = targetPositions(counter2,4);
%                                 counter2 = counter2+1;
%                             end
                        end
                            [out,MAV_loop] = gotoTarget(MAV_loop,sim); %--determine required velocity
                            MAV_loop.u1 = out(1);
                            MAV_loop.u2 = out(2);
                       
                end
                MAV_loop = agentDynamics_B(MAV_loop, sim);
                agent(i) = MAV_loop;
%                 agent(i) = fuel(agent(i),sim);    %This updates the agents fuel levels. Comment out for no refuelling
 
                depot.charge = sum([agent(:).state] == 3)+ (sum([agent(:).state] == 4));
                depot.inUse = sim.numAgents-sum([agent(:).workBool]);
            end
        end
        recPos(t/sim.ts+1,:) = [agent(1).posX agent(1).posY agent(2).posX agent(2).posY];
        [MS_temp,tickCount] = visited_B(grid_MS,MS_temp,agent,tickCount,sim);

        %--Update performance indices
%         100-calcTotalAge_B(MS_temp)/grid_MS.numCells
        aveAge(itt,t/sim.ts+1) = 100-(calcTotalAge_B(MS_temp)/grid_MS.numCells);
        if coverTime(itt) == 0 && sum(tickCount(:) == 0) == 0
            coverTime(itt) = t;
        end

        coverageCount(itt,t/sim.ts+1) = min(tickCount(tickCount>=0));
        coveragePerc(itt,t/sim.ts+1) = (grid_MS.numCells-sum(tickCount(:) == 0)) / grid_MS.numCells*100;
        
        minDistance = 5;    %--This is the threshold for when the penalty is applied given in the article
            for i=1:sim.numAgents   %--This finds the minimum distance between any two agents
                for agentCount=1:sim.numAgents
                    if i ~= agentCount && (agent(agentCount).state == 5 || agent(agentCount).state == 1)
                        distance = sqrt((agent(agentCount).posX-agent(i).posX)^2+(agent(agentCount).posY-agent(i).posY)^2);
                        if distance < minDistance
                            minDistance = distance;
                        end
                    end
                end   
            end
            interUAVDist(floor(t/sim.ts+1)) = minDistance;
            if minDistance <= 0.3
                crashed(itt,floor(t/sim.ts+1)) = 1;
            end
            
            
%         dist = 5;
%         for i = 1:sim.numAgents
%             for j=1:sim.numAgents
%                 if j~= i
%                     distTemp = sqrt((agent(i).posX-agent(j).posX)^2+(agent(i).posY-agent(j).posY)^2); %Between 1 and 2
%                     if distTemp < dist
%                         dist = distTemp;
%                     end
%                 end
%             end
%         end
%         interUAVdist(floor(t/sim.ts+1),1) = dist;
        
%         mesh(MS_temp);
%         drawnow
%         pause(0.005)
        
%         plot([grid_MS.width grid_MS.width],[0 grid_MS.bredth],'r');
%     hold on
%     plot([0 grid_MS.width],[grid_MS.bredth grid_MS.bredth],'r');
%     plot([0 grid_MS.width],[0 0],'r');
%     plot([0 0],[0 grid_MS.bredth],'r');
%     temp = [grid_MS.res grid_MS.res;grid_MS.res (grid_MS.bredth-grid_MS.res);(grid_MS.width-grid_MS.res) (grid_MS.bredth-grid_MS.res);(grid_MS.width-grid_MS.res) grid_MS.res;grid_MS.res grid_MS.res];
%     plot(temp(:,1),temp(:,2),'b')
%         plot(agent(1).posX,agent(1).posY,'bo')
%         plot(agent(2).posX,agent(2).posY,'ro')
%         plot(agent(3).posX,agent(3).posY,'ko')
%         plot(agent(4).posX,agent(4).posY,'go')
%         plot(agent(5).posX,agent(5).posY,'mo')
%         plot(agent(6).posX,agent(6).posY,'o','Color',[160/255 32/255 240/255])
%         plot(agent(7).posX,agent(7).posY,'c+')
%         if t <= t_crash
%             plot(agent(8).posX,agent(8).posY,'+','Color',[160/255 100/255 150/255])
%         end
%         hold off
%         drawnow
%         pause(0.01)

        if t==t_crash
            sim.numAgents = 7;
            agent = agent(1:7);
        end  
        if t==t_crash2
            sim.numAgents = 6;
            agent = agent(1:6);
        end
%        tempFuel(t/sim.ts+1,:) = [agent(1).fuel agent(2).fuel agent(3).fuel agent(4).fuel agent(5).fuel agent(6).fuel]; 
    end%--End sim time for   
    
end%--End itterations for

%--Writing the data to .csv file
time = 0:sim.ts:sim.time;
% csvwrite('baseline_RW2_avoidNewDyn_slow_Age_5050_8.csv',[time',aveAge']);
% csvwrite('baseline_RW2_avoidNewDyn_slow_Cov_5050_8.csv',[time',coveragePerc']);
% csvwrite('baseline_RW2_avoidNewDyn_slow_Crash_5050_8.csv',[time',crashed']);
% csvwrite('baseline_RW2_avoidNewDyn_slow_Age_robust2_8.csv',[time',aveAge']);
% csvwrite('baseline_RW2_avoidNewDyn_slow_Cov_robust2_8.csv',[time',coveragePerc']);
% csvwrite('baseline_RW2_avoidNewDyn_slow_Crash_robust2_8.csv',[time',crashed']);
% csvwrite('baseline_RW2_refuelNewDyn_slow_Age_600.csv',[time',aveAge']);
% csvwrite('baseline_RW2_refuelNewDyn_slow_Cov_600.csv',[time',coveragePerc']);
% csvwrite('baseline_RW2_refuelNewDyn_slow_Crash_600.csv',[time',crashed']);


%--Plotting figures
figure()
plot(time,tempFuel(:,1),'k','linewidth',1)
hold on
plot(time,tempFuel(:,2),'c','linewidth',1)
plot(time,tempFuel(:,3),'r','linewidth',1)
plot(time,tempFuel(:,4),'b','linewidth',1)
plot(time,tempFuel(:,5),'g','linewidth',1)
plot(time,tempFuel(:,6),'y','linewidth',1)

figure()
plot(interUAVDist)

figure()
surface(X,Y,tickCount)
colorbar
title('Number of cell visits')
xlabel('Mission space length')
ylabel('Mission space width')
set(gca,'fontsize',Font_size)
%% -- Figure for the avaoidance movement

figure()
plot(recPos(1:196,1),recPos(1:196,2),'r','linewidth',1)
hold on
plot(recPos(1:196,3),recPos(1:196,4),'k','linewidth',1)
legend('MAV 1','MAV 2');
%--marking the start points of the avoidance 
plot(recPos(29,1),recPos(29,2),'r*','linewidth',1)
plot(recPos(29,3),recPos(29,4),'k*','linewidth',1)
plot(recPos(44,1),recPos(44,2),'r*','linewidth',1)
plot(recPos(44,3),recPos(44,4),'k*','linewidth',1)
plot(recPos(127,1),recPos(127,2),'r*','linewidth',1)
plot(recPos(126,3),recPos(126,4),'k*','linewidth',1)
plot(recPos(141,1),recPos(141,2),'r*','linewidth',1)
plot(recPos(141,3),recPos(141,4),'k*','linewidth',1)
%--Marking the end points
plot(recPos(196,1),recPos(196,2),'ro','linewidth',1)
plot(recPos(196,3),recPos(196,4),'ko','linewidth',1)
plot(recPos(1,1),recPos(1,2),'ro','linewidth',1)
plot(recPos(1,3),recPos(1,4),'ko','linewidth',1)

xlabel('Mission space length');
ylabel('Mission space width')
title('Avoidance Maneuver')
set(gca,'fontsize',Font_size)

