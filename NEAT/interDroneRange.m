%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- simulatedRun
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to find the ranges to any other UAVs that are
% within sensing range of the given UAV. This is given as four ranges
% representing the front, back left and right sensors.
% Date created: 26 June 2018
%
%
%% ----------------

function [uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left] = interDroneRange(agent,index)
% interDroneRange      Calculate the range to the other UAVs
%
% Syntax:              [uavRange_Front,uavRange_Right,uavRange_Back,uavRange_Left] = interDroneRange(agent,index)
%
% Inputs:               
%   agent                   -   array of MAV structures
%   index                   -   index of the current UAV
%
% Outputus:
%   uavRange_Front          -   Range to a UAV in front of currrent UAV
%   uavRange_Right          -   Range to a UAV to the right of currrent UAV
%   uavRange_Back           -   Range to a UAV behinf of currrent UAV
%   uavRange_Left           -   Range to a UAV to the left of currrent UAV

%-- Assigning the default range values
uavRange_Front = agent(index).sensorRange;
uavRange_Right = agent(index).sensorRange;
uavRange_Back = agent(index).sensorRange;
uavRange_Left = agent(index).sensorRange;


for i=1:size(agent,2)
    if i ~= index
        %--Calculating the distances and angles to the other UAVs
        distance = sqrt((agent(index).posX-agent(i).posX)^2+(agent(index).posY-agent(i).posY)^2);
        angle = atand((agent(i).posX-agent(index).posX)/(agent(i).posY-agent(index).posY));
        if agent(i).posX >= agent(index).posX
            if agent(i).posY >= agent(index).posY
                angle = angle;
            else
                angle = 180+angle;
            end
        else
            if agent(i).posY >= agent(index).posY
                angle = 360+angle;
            else
                angle = 180+angle;
            end
        end%--Correcting the angle to account for 360 deg
        
        %--Getting angle to the obstacle relative to the agents heading
        angle = angle - agent(index).head; 
        if angle < 0
            angle = 360+angle;
        end
        
        %--This next stage assigns the obstacles to sensing zones based on the
        %MAVs heading and assigns the geofence value to the min distance in
        %each zone

        %--Front sensor
        if (angle >= 315 || angle < 45) && distance < uavRange_Front
            uavRange_Front = distance;
        end
        %--Right sensor
        if angle >= 45 && angle < 135 && distance < uavRange_Right
            uavRange_Right = distance;
        end
        %--back sensor
        if angle >= 135 && angle < 225 && distance < uavRange_Back
            uavRange_Back = distance;
        end
        %--Left sensor
        if angle >= 225 && angle < 315 && distance < uavRange_Left
            uavRange_Left = distance;
        end
        
    end
end

%Normalising the values
uavRange_Front = uavRange_Front/agent(index).sensorRange;
uavRange_Back = uavRange_Back/agent(index).sensorRange;
uavRange_Left = uavRange_Left/agent(index).sensorRange;
uavRange_Right = uavRange_Right/agent(index).sensorRange;



end

