%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- getMS_ArrayIndex
%
%   This function uses the agents given x,y position and the parametes 
%   stored in the grid structure to determine in which cell in the MS the
%   agent currently is
%
%
%% ----------------

function [x_index, y_index] = getMS_ArrayIndex_B(agent, grid)
% getMS_ArrayIndex      Determines which cell in the MS the agent currently
%                       is in
%
% Syntax:              [x_index, y_index] = getMS_ArrayIndex(agent, grid)
%
% Inputs:               
%   agent           -   structure containing current agent parameters
%   grid            -   structure containing grid parameters
%
% Outputus:
%   x_index         -   cells x index
%   y_index         -   cells y index

x_index = ceil(agent.posX/grid.res);
y_index = ceil(agent.posY/grid.res);

end

