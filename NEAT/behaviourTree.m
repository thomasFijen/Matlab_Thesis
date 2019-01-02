function [ state ] = behaviourTree( agent,depot,index_agent,grid,sim )
%State:
%   1 = Avoidance
%   2 = Homing
%   3 = waiting
%   4 = refueling
%   5 = Surveillance

threshold1 = 50;       % Fuel level at which refuelling can be possible
thresholdCritic = 34;  % Critical fuel level. Land if this is reached

%--Real test
% threshold1 = 40;       % Fuel level at which refuelling can be possible
% thresholdCritic = 15;  % Critical fuel level. Land if this is reached

distThresh = 0.2;       % Distance to depot threshold

state = 5;  % The default state is set to surveillance

dist = 5;
for j=1:sim.numAgents
    if j~= index_agent
        distTemp = sqrt((agent(index_agent).posX-agent(j).posX)^2+(agent(index_agent).posY-agent(j).posY)^2); %Between 1 and 2
        if distTemp < dist
            dist = distTemp;
        end
    end
end


if agent(index_agent).posX <= grid.width && agent(index_agent).posX >= 0 && agent(index_agent).posY <= grid.bredth && agent(index_agent).posY >=0
    if agent(index_agent).state == 4 && agent(index_agent).fuel < 100
        state = 4;
    else
        if (agent(index_agent).fuel < threshold1) && (agent(index_agent).fuel < thresholdCritic || agent(index_agent).state == 2 || depot.inUse == 0)
            if sqrt((agent(index_agent).posX-depot.posX)^2+(agent(index_agent).posY-depot.posY)^2) > distThresh
                state = 2;
            else
                if depot.charge == 0
                    state = 4;
                else
                    state = 3;
                end
            end
        else
            if dist < 0.8
                state = 1;
            elseif dist <= 1.2 && agent(index_agent).state == 1
                state = 1;
            end
        end
    end
else
    state = 2;
end

end


            
