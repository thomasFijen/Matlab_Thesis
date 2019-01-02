%%  MSc Thesis
% Thomas Fijen, 4620852
%% ----------------- simulatedRun
%
% This script forms part of my MSc thesis project entitled: Persistent
% Surveillance of a Greenhouse
% This fuction calculates the total cumulative age of the mission space for
% all non obstacle, cells
% Date created: 9 March 2018
%
%
%% ----------------

function totAge = calcTotalAge_B(MS)
% calcTotalAge         calculates the total cumulative age of the MS
%
% Syntax:              totAge = calcTotalAge(MS)
%
% Inputs:               
%   MS          -  Mission space grid. This could also be a subset of the
%                   MS
%
% Outputus:
%   totAge      - total/cumulative Age of the give MS
%   
    totAge = 0;
    
    for i=1:size(MS,1)
        for j=1:size(MS,2)
            if MS(i,j) >= 0
                totAge = totAge + MS(i,j);
            end
        end
    end


end

