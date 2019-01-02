function [ outRes,agent ] = avoidance(agent,sim,grid_MS,depot)
%This function implements the evolved avoidance strategry developed by T
%Szabo in his thesis.

%--Finding the min distance
% dist = 5;
% for i=1:sim.numAgent
%     if i ~= idx
%         distTemp = sqrt((agent(idx).posX-agent(i).posX)^2+(agent(idx).posY-agent(i).posY)^2);
%         if dist > distTemp 
%             dist = distTemp;
%         end
%     end
% end
angle = pi/4;

% if agent(idx).posX >= 0 && agent(idx).posX <= grid_MS.width && agent(idx).posY >= 0 && agent(idx).posY <= grid_MS.bredth
% if agent.posX >= 0 && agent.posX <= grid_MS.width && agent.posY >= 0 && agent.posY <= grid_MS.bredth
%     if (agent.avoiding == 0)
%         %Find current direction of travel (w.r.t x axis)
%         if agent.velX ~= 0
%             theta = atan(agent.velY/agent.velX);
%         else
%             if agent.velY >= 0
%                 theta = pi/2;
%             else
%                 theta = -pi/2;
%             end
%         end
%         % Reverse direction of travel
%         if agent.velX >= 0
%             out(1) = agent.maxVel*cos(theta);
%             out(2) = agent.maxVel*sin(theta);
%         else
%             out(1) = -agent.maxVel*cos(theta);
%             out(2) = -agent.maxVel*sin(theta);
%         end
%         agent.avoiding = 1;
%         out(1) = -out(1);
%         out(2) = -out(2);
%     else
%         if agent.avoiding == 1
%             %Adjust line by 'angle'
%             if agent.velX ~= 0
%                 theta = atan(agent.velY/agent.velX);
%             else
%                 if agent.velY >= 0
%                     theta = pi/2;
%                 else
%                     theta = -pi/2;
%                 end
%             end
%             %--New direction of travel
%             theta = theta - angle;
% 
%             if agent.velX >= 0
%                 if theta < - pi/2
%                     theta = pi+theta;
%                     out(1) = -agent.maxVel*cos(theta);
%                     out(2) = -agent.maxVel*sin(theta);
%                 else
%                     out(1) = agent.maxVel*cos(theta);
%                     out(2) = agent.maxVel*sin(theta);
%                 end
%             else
%                 if theta < -pi/2
%                     theta = pi+theta;
%                     out(1) = agent.maxVel*cos(theta);
%                     out(2) = agent.maxVel*sin(theta);
%                 else
%                     out(1) = -agent.maxVel*cos(theta);
%                     out(2) = -agent.maxVel*sin(theta);
%                 end
%             end
%             agent.avoiding = 2;
%         else
%             out(1) = agent.u1;
%             out(2) = agent.u2;           
%         end
% %             out(1) = agent.u1;
% %             out(2) = agent.u2; 
%     end
% else
%     %If outside MS return to depot
%     out = homing(agent,depot,sim);
%     agent.avoiding = 0;
% end

if agent.posX >= 0 && agent.posX <= grid_MS.width && agent.posY >= 0 && agent.posY <= grid_MS.bredth
    if (agent.avoiding == 0)
        %Find current direction of travel (w.r.t x axis)
        if agent.u1 ~= 0
            theta = atan(agent.u2/agent.u1);
        else
            if agent.u2>= 0
                theta = pi/2;
            else
                theta = -pi/2;
            end
        end
        % Reverse direction of travel
        if agent.u1 >= 0
            out(1) = agent.maxVel*cos(theta);
            out(2) = agent.maxVel*sin(theta);
        else
            out(1) = -agent.maxVel*cos(theta);
            out(2) = -agent.maxVel*sin(theta);
        end
        agent.avoiding = 1;
        out(1) = -out(1);
        out(2) = -out(2);
    else
        if agent.avoiding == 1
            %Adjust line by 'angle'
            if agent.u1 ~= 0
                theta = atan(agent.u2/agent.u1);
            else
                if agent.u2 >= 0
                    theta = pi/2;
                else
                    theta = -pi/2;
                end
            end
            %--New direction of travel
            theta = theta - angle;

            if agent.u1 >= 0
                if theta < - pi/2
                    theta = pi+theta;
                    out(1) = -agent.maxVel*cos(theta);
                    out(2) = -agent.maxVel*sin(theta);
                else
                    out(1) = agent.maxVel*cos(theta);
                    out(2) = agent.maxVel*sin(theta);
                end
            else
                if theta < -pi/2
                    theta = pi+theta;
                    out(1) = agent.maxVel*cos(theta);
                    out(2) = agent.maxVel*sin(theta);
                else
                    out(1) = -agent.maxVel*cos(theta);
                    out(2) = -agent.maxVel*sin(theta);
                end
            end
            agent.avoiding = 2;
        else
            out(1) = agent.u1;
            out(2) = agent.u2;           
        end
%             out(1) = agent.u1;
%             out(2) = agent.u2; 
    end
else
    %If outside MS return to depot
    out = homing(agent,depot,sim);
    agent.avoiding = 0;
end

outRes(1) = out(1);
outRes(2) = out(2);

end

