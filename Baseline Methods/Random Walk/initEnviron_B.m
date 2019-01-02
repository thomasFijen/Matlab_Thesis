%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- initEnviron
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to initialise the mission space 
% Date created: 21 February 2018
%
%
%% ----------------

function [MS,X,Y] = initEnviron_B(grid)

% initEnviron           Initialises the mission space of the assignment
%
% Syntax:               MS = initEnviron(width, bredth, res)
%
% Inputs:               
%   Width   -   width of a rectangular mission space, [m]
%   bredth  -   bredth of a rectangular mission space, [m]
%   res     -   resolution of the discretisation, [m]. NOTE: This is
%               non-zero
%   obst    -   Array containing the coordinates of obstacle cells
%   noMon   -   Array containing the coordinates of cells that do not require
%               monitoring
%
% Outputus:
%   MS      - 2D array of the mission space containing the cell age

temp = 99.75*ones(grid.bredth/grid.res,grid.width/grid.res);

% --  for loops to assign the obstacles and unmonitored area
% use this when there are used entered obstacles. Add obst and noMon to the
% function handle
% for i = 1:size(obst,1)
%     temp(obst(i,1),obst(i,2)) = -1;
% end
% for i = 1:size(noMon,1)
%     temp(noMon(i,1),noMon(i,2)) = 0;
% end

% -- creating (predefined) obstacles and unmonitored areas
temp(1,:) = -1;
temp(end,:) = -1;
temp(:,1) = -1;
temp(:,end) = -1;

% temp(floor(size(temp,1)/2),floor(size(temp,2)/2)) = 0;
% temp(floor(size(temp,1)/2),floor(size(temp,2)/2)+1) = 0;
% temp(floor(size(temp,1)/2)+1,floor(size(temp,2)/2)) = 0;
% temp(floor(size(temp,1)/2)+1,floor(size(temp,2)/2)+1) = 0;


% -- Assigning outputs
MS = temp;
X = grid.res/2:grid.res:grid.width-grid.res/2;
Y = grid.res/2:grid.res:grid.bredth-grid.res/2;

end %-- end of function initEnviron