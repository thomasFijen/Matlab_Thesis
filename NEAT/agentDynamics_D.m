%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- agentDynamics
%
%   This function determines the agents positoin based on the given
%   dynamics and inputs
%
%
%% ----------------

function [agent_update] = agentDynamics_D(agent, sim)
% agentDynamics        Updates the agents states depending on the given
%                      inputs
%
% Syntax:              [agent] = agentDynamics(agent, grid)
%
% Inputs:               
%   agent           -   structure containing current agent parameters
%   grid            -   structure containing grid parameters
%
% Outputus:
%   agent_update    - updated agent parameters
%
% NOTE: The agent dynamics are based on the 2nd order velocity model in the
% X and Y directions without axis coupling that can be found in the MSc
% thesis by T. Szabo titled "Autonomous Collision Avoidance for
% Swarms of MAVs"

agent_update = agent;

zeta = 58.65;    % Determined experimentally
omega = 14.5194;   % Determined experimentally
tau = 0.3;
kp = 12;

A = [-2*zeta*omega -omega^2 0 0 0 0 0;
    1 0 0 0 0 0 0;
    0 0 -2*zeta*omega -omega^2 0 0 0;
    0 0 1 0 0 0 0;
    0 1 0 0 0 0 0;
    0 0 0 1 0 0 0;
    0 0 0 0 0 0 -1/tau];

B = [omega^2 0 0;
    0 0 0;
    0 omega^2 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 1/tau];

% C = [0 0 0 0 1 0 0;0 0 0 0 0 1 0];
% D = 0;
% 
% sys = ss(A,B,C,D,grid.ts);

%-- Dicretising the system for the given sample time. Uses a zero order
%hold
[Phi,Gamma] = c2d(A,B,sim.ts);

%-- Assigning the current states and the input
X = [agent.accX; agent.velX; agent.accY; agent.velY; agent.posX; agent.posY; agent.height];

U = [kp*(agent.u1-X(2));kp*(agent.u2-X(4));agent.hc];


X_new = Phi*X+Gamma*U;

agent_update.accX = X_new(1);
agent_update.velX = X_new(2);
agent_update.accY = X_new(3);
agent_update.velY = X_new(4);
agent_update.posX = X_new(5);
agent_update.posY = X_new(6);
agent_update.height = X_new(7);

end %--end of agentDynamics function

