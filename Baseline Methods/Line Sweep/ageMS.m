%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- ageMS
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This function is called to age the mission space by one timestep
% Date created: 21 February 2018
%
%
%% ----------------

function MS_new = ageMS(ts,MS_old)

% ageMS                Initialises the mission space of the assignment
%
% Syntax:              MS_new = ageMS(ts,MS_old)
%
% Inputs:               
%   ts          -   time step, [s]
%   MS_old      -   copy of the old mission space
%
% Outputus:
%   MS_new      - updated mission space

for i=1:size(MS_old,1)
    for j = 1:size(MS_old,2)
        if MS_old(i,j) ~= 0 && MS_old(i,j) ~= -1
            MS_old(i,j) = MS_old(i,j) + ts;
        end
    end
end

MS_new = MS_old;

end