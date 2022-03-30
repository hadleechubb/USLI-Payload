function [cellID] = FindCell(finalLocation) %Takes argument of vector containing x, y, z coordinate

xOff = -250; %x and y offset of launchpad from gridded image origin. Enter the x and y components of the distance offset.
yOff = 250;

xLoc = floor((finalLocation(1)+xOff)/250); %Finds the upper right-hand node of the cell that contains the location of the rocket
yLoc = ceil((finalLocation(2)+yOff)/250);

cellID = 211 + xLoc - 20*yLoc %Traces from origin to determine the cell number. Can be calculated.

end
