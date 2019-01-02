function [topLeft, topRight,bottomLeft,bottomRight] = weightedAveMS_D( grid,MS,agent )
% weightedAveMS_D      Calculates the cumulative age of the cells around  
%                      the given agent. This is used as an input to the NN.
%                       This is done in terms of the global coordinate
%                       frame, not the body frame.
%
% Syntax:              weightedAveMS_D( grid,MS,agent )
%
% Inputs:               
%   grid                 -   size of initial population
%   MS                   -   Number of inputs to the ANN
%   agent                -   Number of outputs for the ANN
%
% Outputus:
%   topLeft              -   Ages of the cells to the left and above the
%                               agents current position
%   topRight             -   Ages of the cells to the rigth and above the
%                               agents current position
%   bottomLeft           -   Ages of the cells to the left and below the
%                               agents current position
%   bottomRight          -   Ages of the cells to the right and below the
%                               agents current position
%   

    %if agent.posX >= 0 && agent.posX <= grid.width && agent.posY >= 0 && agent.posY <= grid.bredth
    
    %--Finding the currently occupied cell
    currentY = ceil((agent.posY)/grid.res);
    currentX = ceil((agent.posX)/grid.res);
    
    %--Determining the total age of the cells above the agent
    if currentY < ceil((grid.bredth)/grid.res)
        if currentY >= 1 && currentX >= 1 && currentX <= ceil((grid.width)/grid.res)
            subsetTL = MS(currentY:ceil((grid.bredth)/grid.res),1:currentX);
            subsetTR = MS(currentY:ceil((grid.bredth)/grid.res),currentX:ceil((grid.width)/grid.res));
            
            subsetBL = MS(1:currentY,1:currentX);
            subsetBR = MS(1:currentY,currentX:ceil((grid.width)/grid.res));
            
            topLeft = calcTotalAge_D(subsetTL);
            topRight = calcTotalAge_D(subsetTR);
            bottomLeft = calcTotalAge_D(subsetBL);
            bottomRight = calcTotalAge_D(subsetBR);
        elseif currentY >= 1 && currentX < 1 
            subsetTR = MS(currentY:ceil((grid.bredth)/grid.res),1:ceil((grid.width)/grid.res));
            
            subsetBR = MS(1:currentY,1:ceil((grid.width)/grid.res));
            
            topLeft = 0;
            topRight = calcTotalAge_D(subsetTR);
            bottomLeft = 0;
            bottomRight = calcTotalAge_D(subsetBR);
        elseif currentY >= 1 && currentX > ceil((grid.width)/grid.res) 
            subsetTL = MS(currentY:ceil((grid.bredth)/grid.res),1:ceil((grid.width)/grid.res));
            
            subsetBL = MS(1:currentY,1:ceil((grid.width)/grid.res));
            
            topLeft = calcTotalAge_D(subsetTL);
            topRight = 0;
            bottomLeft = calcTotalAge_D(subsetBL);
            bottomRight = 0;
       %-Cases where the agent is 'below' the MS     
        elseif currentY < 1 && currentX >= 1 && currentX <= ceil((grid.width)/grid.res)
            subsetTL = MS(1:ceil((grid.bredth)/grid.res),1:currentX);
            subsetTR = MS(1:ceil((grid.bredth)/grid.res),currentX:ceil((grid.width)/grid.res));
            
            topLeft = calcTotalAge_D(subsetTL);
            topRight = calcTotalAge_D(subsetTR);
            bottomLeft = 0;
            bottomRight = 0;
        elseif currentY < 1 && currentX < 1 
            subsetTR = MS(1:ceil((grid.bredth)/grid.res),1:ceil((grid.width)/grid.res));
            
            topLeft = 0;
            topRight = calcTotalAge_D(subsetTR);
            bottomLeft = 0;
            bottomRight = 0;
        elseif currentY < 1 && currentX > ceil((grid.width)/grid.res) 
            subsetTL = MS(1:ceil((grid.bredth)/grid.res),1:ceil((grid.width)/grid.res));
            
            topLeft = calcTotalAge_D(subsetTL);
            topRight = 0;
            bottomLeft = 0;
            bottomRight = 0;
        end
    else
        %--case when there are no cells from the MS above the agent (global coordinate frame)
        topLeft = 0;
        topRight = 0;
        
        if currentX >= 1 && currentX <= ceil((grid.width)/grid.res)
            subsetBL = MS(1:ceil((grid.bredth)/grid.res),1:currentX);
            subsetBR = MS(1:ceil((grid.bredth)/grid.res),currentX:ceil((grid.width)/grid.res));

            bottomLeft = calcTotalAge_D(subsetBL);
            bottomRight = calcTotalAge_D(subsetBR);
        elseif currentX < 1 
            subsetBR = MS(1:ceil((grid.bredth)/grid.res),1:ceil((grid.width)/grid.res));
            
            bottomLeft = 0;
            bottomRight = calcTotalAge_D(subsetBR);
        elseif currentX > ceil((grid.width)/grid.res) 
            subsetBL = MS(1:ceil((grid.bredth)/grid.res),1:ceil((grid.width)/grid.res));
            
            bottomLeft = calcTotalAge_D(subsetBL);
            bottomRight = 0;
        end
    end
    
    %-- Normalise the values 
    topLeft = (grid.numCells*100-topLeft)/(grid.numCells*100);
    topRight = (grid.numCells*100-topRight)/(grid.numCells*100);
    bottomLeft = (grid.numCells*100-bottomLeft)/(grid.numCells*100);
    bottomRight = (grid.numCells*100-bottomRight)/(grid.numCells*100);

end

