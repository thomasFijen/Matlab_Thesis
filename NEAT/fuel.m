%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- simulatedRun
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to update the current fuel level of the given MAV.
% Date created: 18 July 2018
%
%
%% ----------------

function [ agent ] = fuel( agent,sim)
% simulatedRun         Simulates the mission over time horizon
%
% Syntax:              MS_new = simulatedRun(ts,MS_old)
%
% Inputs:               
%   agent                   -   MAV structure
%   grid                    -   parrameters of the MS 
%   refuel_bool             -   currently refuelling boolean 
%
% Outputs:
%   agent                   -   updated MAV structure
%
%--Assumptions:
%     - This function assumes that the MAV discharges at a constant rate
%       while in use
%     - The MAV is rechared at a constant rate while in the recharge cell

if agent.state ~= 4 && agent.state ~= 3
    agent.fuel = agent.fuel - agent.dischargeRate * sim.ts;
    if agent.fuel <= 0
            agent.crashed = 1;
            disp('Agent Crashed due to lack of Fuel')
            disp(['Pos X = ',num2str(agent.posX),', Pos Y = ',num2str(agent.posY)]);
    end
else
    if agent.state == 4
        if agent.fuel > 0
            agent.fuel = agent.fuel + agent.refuelRate * sim.ts;
        end
        if agent.fuel > 100
            agent.fuel = 100;
        end
        
    elseif agent.state == 3
        agent.fuel = agent.fuel;
    end
    
end

end

