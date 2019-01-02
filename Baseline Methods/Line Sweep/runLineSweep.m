%% MSc Thesis:
% Thomas Fijen, 4620852

%% ------------ runLineSweep
%   This script implements the basic line sweep coverage planner
%   Date: 21 Feb 2018

clc
clear
close all

Font_size = 18;

set(0,'DefaultAxesXGrid','on','DefaultAxesYGrid','on',...
    'DefaultAxesZGrid','on');
set(0,'defaultFigurePosition',[488 100 560*1.6 420*1.4]);
set(0,'defaultFigurePaperPositionMode','auto');


%% ----------- Setup the mission space

grid_MS.width = 25;             % width of rectangular mission space in [m]
grid_MS.bredth  = 25;           % bredth of rectangular mission space in [m] (Actually height)
grid_MS.res = 1;                % resolution of the discretisation
grid_MS.posTol = 0.1;           % positional tolerance of the agents
grid_MS.numCells = 0;           % Number of non obstacle cells

[MS,X,Y] = initEnviron_B(grid_MS);
grid_MS.numX = size(X,2);
grid_MS.numY = size(Y,2);
grid_MS.numCells = sum(MS(:) >= 0); %This counts the number of cells with positive values (i.e: the cells without obstacles)

% ---------- Defining the simulation parameters
sim.numAgents = 8;
sim.time = 300;             % Total simulation time in [s] 
sim.ts = 0.25;               % Size of the time steps in [s]
t_crash = 400;
t_crash2 = 400;
grid_MS.regions = zeros(sim.numAgents, 4); % Contains the borders of the subregions for each drone [left rigth top bottom] 
grid_MS.start = zeros(sim.numAgents, 2); % Start positions of the agents (global ref) [posX posY]

% grid.regions = [1,12,24,1;12,24,24,1]; 
% grid.start = [1.5 1.5;12.5 1.5];

%--Assigning the agents to their own sub region
grid_MS.regions = [12,18,24,12;
                18,24,24,12;
                18,24,12,1;
                12,18,12,1;
                6,12,12,1;
                1,6,12,1;
                1,6,24,12;
                6,12,24,12];
            
grid_MS.start = [12.5 12.5;
                18.5 12.5;
                18.5 1.5;
                12.5 1.5;
                6.5 1.5;
                1.5 1.5;
                1.5 12.5;
                6.5 12.5];

%         grid_MS.regions = [8,16,24,12;
%                 16,24,24,12;
%                 16,24,12,1;
%                 8,16,12,1;
%                 1,8,12,1;
%                 1,8,24,12];
%             
%         grid_MS.start = [8.5 12.5;
%                 16.5 12.5;
%                 16.5 1.5;
%                 8.5 1.5;
%                 1.5 1.5;
%                 1.5 12.5];

% ---------- Defining the agents
for i=1:sim.numAgents
    MAV(i).posX = grid_MS.width/2;        % [m]
    MAV(i).posY = grid_MS.bredth/2;       % [m]
    MAV(i).velX = 0;                % [m/s]
    MAV(i).accX = 0;                % [m/s^2]
    MAV(i).velY = 0;                % [m/s]
    MAV(i).accY = 0;                % [m/s^2]
    MAV(i).height = 1.5;            % [m]
    MAV(i).vel = 1;                 % Commanded velocity, [m/s]
    MAV(i).head = 0;                % Commanded heading, [rad]
    MAV(i).hc = 1.5;                % Command height, [m]
    MAV(i).targetX = grid_MS.start(i,1);           % [m]
    MAV(i).targetY = grid_MS.start(i,2);           % [m]
    MAV(i).targetFlag = 1;          % 1 if moving to assigned target cells
    MAV(i).u1 = 0;                  % This is the first output of the NN. Corresponds to: Velocity X
    MAV(i).u2 = 0;                  % This is the second output of the NN. Coresponds to: Velocity Y
    MAV(i).sensorRange = 10;        % Sensor range in [m]
    MAV(i).maxVel = 0.5;            % Maximum velocity of the agent [m/s]
    MAV(i).footprint = 2.5;         % Radius of the sensor footprint [m]
    
    MAV(i).turnFlag = 0;
    MAV(i).refuelRate = 10;         % Rate at which the UAV refuels itself while charging, [/s]
    MAV(i).workBool = 1;
    MAV(i).chargeBool = 0;          % Boolean, is MAV recharging
    MAV(i).waitBool = 0;
    MAV(i).fuel = 100;              % This represents the fuel level with 100 being fully charged and 0 as empty
    MAV(i).dischargeRate = 1;       % Rate at which fuel is consumed, [unit/s] 
    MAV(i).crashed = 0;             % Boolean stating whether the MAV has crashed
end

%% ---------- TEST: line sweep

    ittOffset = 2;
    itt = 1;
    MAV(1).posX = MAV(1).posX;
    MAV(1).posY = MAV(1).posY + itt*ittOffset;
    MAV(2).posX = MAV(2).posX + itt*ittOffset*cosd(45);
    MAV(2).posY = MAV(2).posY + itt*ittOffset*sind(45);
    MAV(3).posX = MAV(3).posX+ itt*ittOffset;
    MAV(3).posY = MAV(3).posY;
    MAV(4).posX = MAV(4).posX + itt*ittOffset*cosd(45);
    MAV(4).posY = MAV(4).posY - itt*ittOffset*sind(45);
    MAV(5).posX = MAV(5).posX;
    MAV(5).posY = MAV(5).posY - itt*ittOffset;
    MAV(6).posX = MAV(6).posX - itt*ittOffset*cosd(45);
    MAV(6).posY = MAV(6).posY - itt*ittOffset*sind(45);
    MAV(7).posX = MAV(7).posX - itt*ittOffset;
    MAV(7).posY = MAV(7).posY;
    MAV(8).posX = MAV(8).posX - itt*ittOffset*cosd(45);
    MAV(8).posY = MAV(8).posY + itt*ittOffset*sind(45);

% %-- Plotting the age map
% figure()
% mesh(X,Y,MS)
% colorbar;
% xlabel('Mission space width');
% ylabel('Mission space bredth')
% view(-45,45)
% hold on

%-- Plotting the positions
figure()
plot(MAV(1).posX,MAV(1).posY,'bo')
hold on
plot(MAV(2).posX,MAV(2).posY,'ro')
plot(MAV(3).posX,MAV(3).posY,'ko')
plot(MAV(4).posX,MAV(4).posY,'go')
plot(MAV(5).posX,MAV(5).posY,'b+')
plot(MAV(6).posX,MAV(6).posY,'r+')
plot(MAV(7).posX,MAV(7).posY,'k+')
plot(MAV(8).posX,MAV(8).posY,'g+')
xlabel('Mission space width');
ylabel('Mission space bredth')
set(gca,'fontsize',Font_size)


%--Testing data
tickCount = MS;

aveAge = zeros(1,sim.time/sim.ts+1);
coveragePerc = zeros(1,sim.time/sim.ts+1);
        
for t=0:sim.ts:sim.time
    for i=1:sim.numAgents
        if MAV(i).targetFlag == 0
            MAV(i) = lineSweep(MAV(i), MS, grid_MS,i);
            if i == 7
                MAV(i).targetY
            end
        end 

        [out,MAV(i)] = gotoTarget(MAV(i),sim);
        MAV(i).u1 = out(1);
        MAV(i).u2 = out(2);

        MAV(i) = agentDynamics_B(MAV(i), sim);
    end
    
    %-- Plotting the age map
%     mesh(X,Y,MS)
%     title(['Age Map at time: ' num2str(t) ' sec']);
%     colorbar;
%     pause(0.1)
    
    plot(MAV(1).posX,MAV(1).posY,'bo')
    plot(MAV(2).posX,MAV(2).posY,'ro')
    plot(MAV(3).posX,MAV(3).posY,'ko')
    plot(MAV(4).posX,MAV(4).posY,'go')
    plot(MAV(5).posX,MAV(5).posY,'b+')
    plot(MAV(6).posX,MAV(6).posY,'r+')
    
    if t <= t_crash2
        plot(MAV(7).posX,MAV(7).posY,'k+')
    end
    if t <= t_crash
        plot(MAV(8).posX,MAV(8).posY,'g+')
    end
    pause(0.01)
    
    
%     MS = ageMS_B(grid,MS,MAV,sim);
    [MS,tickCount] = visited_B(grid_MS,MS,MAV,tickCount,sim);
    
     %--Update performance indices
    aveAge(t/sim.ts+1) = calcTotalAge_B(MS)/grid_MS.numCells;
    coveragePerc(t/sim.ts+1) = (grid_MS.numCells-sum(tickCount(:) == 0)) / grid_MS.numCells*100;
    
    if t==t_crash
        sim.numAgents = 7;
        MAV = MAV(1:7);
        grid_MS.regions = [8,15,24,14;
                15,24,24,14;
                18,24,14,1;
                12,18,14,1;
                6,12,14,1;
                1,6,14,1;
                1,8,24,14];
            
        grid_MS.start = [8.5 14.5;
                15.5 14.5;
                18.5 1.5;
                12.5 1.5;
                6.5 1.5;
                1.5 1.5;
                1.5 14.5];
        for i=1:7
            MAV(i).targetX = grid_MS.start(i,1);           % [m]
            MAV(i).targetY = grid_MS.start(i,2);           % [m]
            MAV(i).targetFlag = 1;          % 1 if moving to assigned target cells
        end
    end   
    
    if t==t_crash2
        sim.numAgents = 6;
        MAV = MAV(1:6);
        grid_MS.regions = [8,16,24,12;
                16,24,24,12;
                16,24,12,1;
                8,16,12,1;
                1,8,12,1;
                1,8,24,12];
            
        grid_MS.start = [8.5 12.5;
                16.5 12.5;
                16.5 1.5;
                8.5 1.5;
                1.5 1.5;
                1.5 12.5];
        for i=1:6
            MAV(i).targetX = grid_MS.start(i,1);           % [m]
            MAV(i).targetY = grid_MS.start(i,2);           % [m]
            MAV(i).targetFlag = 1;          % 1 if moving to assigned target cells
        end
     end 
    
end
time = 0:sim.ts:sim.time;
% csvwrite('baseline_LS_Age_newDyn_robust1_slow.csv',[time',aveAge']);
% csvwrite('baseline_LS_Cov_newDyn_robust1_slow.csv',[time',coveragePerc']);
csvwrite('baseline_LS_Age_newDyn_slow.csv',[time',aveAge']);
csvwrite('baseline_LS_Cov_newDyn_slow.csv',[time',coveragePerc']);
    
plot([grid_MS.width grid_MS.width],[0 grid_MS.bredth],'r');
    plot([0 grid_MS.width],[grid_MS.bredth grid_MS.bredth],'r');
    plot([0 grid_MS.width],[0 0],'r');
    plot([0 0],[0 grid_MS.bredth],'r');
    temp = [grid_MS.res grid_MS.res;grid_MS.res (grid_MS.bredth-grid_MS.res);(grid_MS.width-grid_MS.res) (grid_MS.bredth-grid_MS.res);(grid_MS.width-grid_MS.res) grid_MS.res;grid_MS.res grid_MS.res];
    plot(temp(:,1),temp(:,2),'b')
hold off

figure()
surface(X,Y,tickCount)
colorbar
title('Number of cell visits')
xlabel('Mission space length')
ylabel('Mission space width')
set(gca,'fontsize',Font_size)
