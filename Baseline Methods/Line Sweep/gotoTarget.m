
function [out,agent] = gotoTarget(agent,sim)
% visited       Motion of agent traveling to a target cell     
%
% Syntax:       MS = visited(MAV,MS,grid)
%
% Inputs:               
%   MAV     -   MAV Structure
%   MS      -   2D mission space age grid
%
% Outputus:
%   MS      -   Updated mission space age grid

%NOTE: Assumptions:
%   -   that there are no obstacles between current point and the target 
%       position.
%   -   Does not overshoot the target
%   -   Movement in 1 timestep is great enough to leave the tolerance area

dist = sqrt((agent.posX-agent.targetX)^2+(agent.posY-agent.targetY)^2);

if agent.targetX > agent.posX
    if agent.targetY > agent.posY
        theta = atan((agent.targetX-agent.posX)/(agent.targetY-agent.posY));
    else
        theta = pi/2+abs(atan((agent.targetY-agent.posY)/(agent.targetX-agent.posX)));
    end
else
    if agent.targetY > agent.posY
        theta = 1.5*pi+abs(atan((agent.targetY-agent.posY)/(agent.targetX-agent.posX)));
    else
        theta = 1.5*pi-abs(atan((agent.targetY-agent.posY)/(agent.targetX-agent.posX)));
    end
end

if dist > (agent.maxVel*sim.ts)
%         if agent.targetY > agent.posY
        out(1) = agent.maxVel*sin(theta);   % x vel
        out(2) = agent.maxVel*cos(theta);   % Y vel
%         else
%             out(1) = agent.maxVel*sin(theta);   % x vel
%             out(2) = agent.maxVel*cos(theta);   % Y vel
%         end
else
%         if agent.targetX > agent.posX
        out(1) = dist*sin(theta)/sim.ts;
        out(2) = dist*cos(theta)/sim.ts;
%         else
%             
%         end
end

if dist < 0.01
	agent.targetFlag = 0;
end

end%-- end function gotoTarget