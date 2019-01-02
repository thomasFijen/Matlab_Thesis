%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- geofence
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to generate the ranges to sensed obstacles on the
% four sides of the given UAV
% Date created: 6 March 2018
%
%
%% ----------------

function [gf_front, gf_back, gf_right, gf_left] = geofence( MS,agent,grid )
%This function calculates the simulated distance to obstacles around the
%agent and the distances to the borders. This function assumes that the MS
%is either square or rectangular.

if agent.posX >= 0 && agent.posX <= grid.width && agent.posY >= 0 && agent.posY <= grid.bredth %If agent is in the MS
    rangeX_max = min(agent.sensorRange,grid.width-agent.posX);
    rangeX_Min = min(agent.sensorRange,agent.posX);
    rangeY_max = min(agent.sensorRange,grid.bredth-agent.posY);
    rangeY_Min = min(agent.sensorRange,agent.posY);

    %--Finding the subset of sensed cells
    indexY_s = ceil((agent.posY-rangeY_Min)/grid.res);
    indexY_e = ceil((agent.posY+rangeY_max)/grid.res);
    indexX_s = ceil((agent.posX-rangeX_Min)/grid.res);
    indexX_e = ceil((agent.posX+rangeX_max)/grid.res);

    if indexY_s == 0 
        indexY_s = 1;
    end
    if indexX_s == 0 
        indexX_s = 1;
    end

    subsetMS = MS(indexY_s:indexY_e,indexX_s:indexX_e);


    [obs_Y, obs_X] = find(subsetMS == -1);
            
    %-These are the default values. This needs to take into account the
    %distance to the borders and the agents heading.
    if (agent.head >= 315 || agent.head < 45)
        gf_front = min(agent.sensorRange,grid.bredth-agent.posY);
        gf_back = min(agent.sensorRange,agent.posY);
        gf_left = min(agent.sensorRange,agent.posX);
        gf_right = min(agent.sensorRange,grid.width-agent.posX);
    elseif agent.head >= 45 && agent.head < 135
        gf_front = min(agent.sensorRange,grid.width-agent.posX);
        gf_back = min(agent.sensorRange,agent.posX);
        gf_left = min(agent.sensorRange,grid.bredth-agent.posY);
        gf_right = min(agent.sensorRange,agent.posY);
    elseif agent.head >= 135 && agent.head < 225
        gf_front = min(agent.sensorRange,agent.posY);
        gf_back = min(agent.sensorRange,grid.bredth-agent.posY);
        gf_left = min(agent.sensorRange,grid.width-agent.posX);
        gf_right = min(agent.sensorRange,agent.posX);
    elseif agent.head >= 225 && agent.head < 315 
        gf_front = min(agent.sensorRange,agent.posX);
        gf_back = min(agent.sensorRange,grid.width-agent.posX);
        gf_left = min(agent.sensorRange,agent.posY);
        gf_right = min(agent.sensorRange,grid.bredth-agent.posY);
    end
%     gf_front = agent.sensorRange;
%     gf_back = agent.sensorRange;
%     gf_left = agent.sensorRange;
%     gf_right = agent.sensorRange;
            
    if (~isempty(obs_X))
        startX = (indexX_s*grid.res-grid.res); %X pos of bottom corner of starting cell of subset MS
        startY = (indexY_s*grid.res-grid.res); %Y pos of bottom corner of starting cell of subset MS

        obsDistance = [0 0]; %the first column contains the distance to the obstacle and the second the heading to obstacle 
            
        for i=1:size(obs_Y,1)
            obs_PosX = startX + obs_X(i)*grid.res-0.5*grid.res; %X position of the identified cell in terms of the global coordinate system
            obs_PosY = startY + obs_Y(i)*grid.res-0.5*grid.res; %Y position of the identified cell in terms of the global coordinate system
            
            %--Determining the distance and angle to the obstacles
            obsDistance(1) = sqrt((obs_PosX-agent.posX)^2+(obs_PosY-agent.posY)^2);
            obsDistance(2) = atand((obs_PosX-agent.posX)/(obs_PosY-agent.posY));

            angle = atand((obs_PosX-agent.posX)/(obs_PosY-agent.posY));
            if obs_PosX >= agent.posX
                if obs_PosY >= agent.posY
                    obsDistance(2) = angle;
                else
                    obsDistance(2) = 180+angle;
                end
            else
                if obs_PosY >= agent.posY
                    obsDistance(2) = 360+angle;
                else
                    obsDistance(2) = 180+angle;
                end
            end

            %Getting angle to the obstacle relative to the agents heading
            obsDistance(2) = obsDistance(2) - agent.head; 
            if obsDistance(2) < 0
                obsDistance(2) = 360+obsDistance(2);
            end

            %--This next stage assigns the obstacles to sensing zones based on the
            %MAVs heading and assigns the geofence value to the min distance in
            %each zone

            %--Front Geofence
            if (obsDistance(2) >= 315 || obsDistance(2) < 45) && obsDistance(1) < gf_front
                gf_front = obsDistance(1);
            end
            %--Right Geofence
            if obsDistance(2) >= 45 && obsDistance(2) < 135 && obsDistance(1) < gf_right
                gf_right = obsDistance(1);
            end
            %--back Geofence
            if obsDistance(2) >= 135 && obsDistance(2) < 225 && obsDistance(1) < gf_back
                gf_back = obsDistance(1);
            end
            %--Left Geofence
            if obsDistance(2) >= 225 && obsDistance(2) < 315 && obsDistance(1) < gf_left
                gf_left = obsDistance(1);
            end
           
        end%--This loop finds the distances and headings to all the sensed obstacles
    end
    
else %Case where the MAV is outside the MS
    
    %--This if statement configures the geofence sensor to "point towards"
    %the MS if the agent moves beyond the MS. This isn't done right. Needs
    %to take the heading into account

    centreAngle = 0;
    angle = atand((grid.width/2-agent.posX)/(grid.bredth/2-agent.posY));
    %--Find angle from agent to centre of MS (Global reference frame)
    if grid.width/2 >= agent.posX
        if grid.bredth/2 >= agent.posY
            centreAngle = angle;
        else
            centreAngle = 180+angle;
        end
    else
        if grid.bredth/2 >= agent.posY
            centreAngle = 360+angle;
        else
            centreAngle = 180+angle;
        end
    end

    %--Getting angle to the centre relative to the agents heading
    centreAngle = centreAngle - agent.head; 
    if centreAngle < 0
        centreAngle = 360+centreAngle;
    end  
    
    %-Assigning the ranges to the correct sensor
    gf_front = 0;
    gf_back = 0;
    gf_left = 0;
    gf_right = 0;

    %--Front Geofence
    if (centreAngle >= 315 || centreAngle < 45) 
        gf_front = agent.sensorRange;
    end
    %--Right Geofence
    if centreAngle >= 45 && centreAngle < 135 
        gf_right = agent.sensorRange;
    end
    %--back Geofence
    if centreAngle >= 135 && centreAngle < 225 
        gf_back = agent.sensorRange;
    end
    %--Left Geofence
    if centreAngle >= 225 && centreAngle < 315 
        gf_left = agent.sensorRange;
    end
   

   
end

%Normalising the values
gf_front = gf_front/agent.sensorRange;
gf_back = gf_back/agent.sensorRange;
gf_left = gf_left/agent.sensorRange;
gf_right = gf_right/agent.sensorRange;

end

