function [cellID] = FindCell(finalLocation) %Takes argument of vector containing x, y, z coordinate

cells = zeros(22, 22); %Initializes cell array - 250 ft x 250 ft

cellName = 1;
    for i = 1:1:size(cells, 1) %Numbers cells top to bottom
        for j = 1:1:size(cells, 2)
           cells(i, j, 1)= cellName;%Names cell
           cellName = cellName + 1; 
        end    
    end

cells
xLoc = floor(finalLocation(1)/250); %Finds the upper right-hand node of the cell that contains the location of the rocket
yLoc = ceil(finalLocation(2)/250);

cellID = 254 + xLoc - size(cells, 2)*yLoc %Traces from origin to determine the cell number. Can be calculated.

xCoord = mod(cellID, size(cells, 2));
yCoord = ceil(cellID/size(cells, 2));

if xCoord == 1
    xCoord = "A";
elseif xCoord == 2
    xCoord = "B";
elseif xCoord == 3
    xCoord = "C";
elseif xCoord == 4
    xCoord = "D";
elseif xCoord == 5
    xCoord = "E";
elseif xCoord == 6
    xCoord = "F";
elseif xCoord == 7
    xCoord = "G";
elseif xCoord == 8
    xCoord = "H";
elseif xCoord == 9
    xCoord = "I";
elseif xCoord == 10
    xCoord = "J";
elseif xCoord == 11
    xCoord = "K";
elseif xCoord == 12
    xCoord = "L";
elseif xCoord == 13
    xCoord = "M";
elseif xCoord == 14
    xCoord = "N";
elseif xCoord == 15
    xCoord = "O";
elseif xCoord == 16
    xCoord = "P";
elseif xCoord == 17
    xCoord = "Q";
elseif xCoord == 18
    xCoord = "R";
elseif xCoord == 19
    xCoord = "S";
elseif xCoord == 20
    xCoord = "T";
elseif xCoord == 21
    xCoord = "U";
elseif xCoord == 22
    xCoord = "V";
end

yCoord = num2str(yCoord);
cellID = strcat(xCoord, yCoord);
end