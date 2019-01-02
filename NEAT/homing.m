function [ out ] = homing( agent,depot,sim )
    
    dist = sqrt((agent.posX-depot.posX)^2+(agent.posY-depot.posY)^2);
    
    if depot.posX > agent.posX
        if depot.posY > agent.posY
            theta = atan((depot.posX-agent.posX)/(depot.posY-agent.posY));
        else
            theta = pi/2+abs(atan((depot.posY-agent.posY)/(depot.posX-agent.posX)));
        end
    else
        if depot.posY > agent.posY
            theta = 1.5*pi+abs(atan((depot.posY-agent.posY)/(depot.posX-agent.posX)));
        else
            theta = 1.5*pi-abs(atan((depot.posY-agent.posY)/(depot.posX-agent.posX)));
        end
    end
    
    if dist > (agent.maxVel*sim.ts)
%         if depot.posY > agent.posY
            out(1) = agent.maxVel*sin(theta);   % x vel
            out(2) = agent.maxVel*cos(theta);   % Y vel
%         else
%             out(1) = agent.maxVel*sin(theta);   % x vel
%             out(2) = agent.maxVel*cos(theta);   % Y vel
%         end
    else
%         if depot.posX > agent.posX
            out(1) = dist*sin(theta)/sim.ts;
            out(2) = dist*cos(theta)/sim.ts;
%         else
%             
%         end
    end

end

