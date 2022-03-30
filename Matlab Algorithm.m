function [time, apogee, location] = absTransform(fileName, frequency, xVel0, yVel0, zVel0, xDist0, yDist0, zDist0, xThe0, yThe0, zThe0) %EXCEL file. Inclute file type in the file name (i.e., '.xls') It should have the x, y, and z acceleration in the 1st, 2nd, and 3rd columns respectively. Include frequency in Hz
    [num1, ~, ~] = xlsread(fileName); %Creates acceleration arrays from the EXCEL file
    
   ts = 0; %number of time stamos removed
   num = zeros(size(num1, 1), 1); %Filters out time stamps!!!
    for i = 1:1:size(num1, 1)
        j = i+ts;
        while mod(j, 6002) == 0 || mod(j, 6002) == 1
            ts = ts + 1;
            j = j + 1;
        end

        num(i, 1) = num1(j, 1);
       if i+ts == size(num1, 1)
           break;
       end
    end
    xAcc = zeros(size(num, 1)/6, 1); %Initializing acceleration arrays
    yAcc = zeros(size(num, 1)/6, 1);
    zAcc = zeros(size(num, 1)/6, 1);
    xAcca = zeros(size(num, 1)/6, 1); %Initializing absolute acceleration arrays
    yAcca = zeros(size(num, 1)/6, 1);
    zAcca = zeros(size(num, 1)/6, 1);
    xOme = zeros(size(num, 1)/6, 1);
    yOme = zeros(size(num, 1)/6, 1);
    zOme = zeros(size(num, 1)/6, 1);
    curtime = zeros(size(num, 1)/6, 1); %Initializing time array
    t = 1/frequency; %Finds time step process
    
    counter = 1;
    for i = 1:6:size(num, 1)
    xAcc(counter, 1) = num(i, 1);
    yAcc(counter, 1) = num(i+1, 1);
    zAcc(counter, 1) = -1*num(i+2, 1);
    xOme(counter, 1) = num(i+3, 1);
    yOme(counter, 1) = num(i+4, 1);
    zOme(counter, 1) = num(i+5, 1);
        if(counter == 1)
            curtime(counter, 1) = 0;
        else
            curtime(counter, 1) = curtime(counter-1, 1)+t;
        end
    counter = counter + 1;
    end
   
    
    xVel = zeros(length(xAcc), 1); %Initializing velocity arrays
    yVel = zeros(length(yAcc), 1);
    zVel = zeros(length(yAcc), 1);
    xVel(1) = xVel0;
    yVel(1) = yVel0;
    zVel(1) = zVel0;
    
    xDist = zeros(length(xVel), 1); %Initializing position arrays
    yDist = zeros(length(yVel), 1);
    zDist = zeros(length(zVel), 1);
    xDist(1) = xDist0;
    yDist(1) = yDist0;
    zDist(1) = zDist0;
    
    xThe = zeros(length(xOme), 1); %Initializing angle arrays
    yThe = zeros(length(yOme), 1);
    zThe = zeros(length(zOme), 1);
    xThe(1) = xThe0;
    yThe(1) = yThe0;
    zThe(1) = zThe0;
    
    for i = 1:1:(length(xOme)-1) %Calculates angle for absolute acceleration
        xThe(i+1) = xThe(i) + 0.5*t*(xOme(i) + xOme(i+1));
        yThe(i+1) = yThe(i) + 0.5*t*(yOme(i) + yOme(i+1));
        zThe(i+1) = zThe(i) + 0.5*t*(zOme(i) + zOme(i+1));
    end
    
    
    for i = 1:1:(length(xAcc)) %Calculates absolute acceleration at each time point
        acca = [cosd(zThe(i)) sind(zThe(i)) 0; -sind(zThe(i)) cosd(zThe(i)) 0; 0 0 1]*[cosd(yThe(i)) 0 -sind(yThe(i)); 0 1 0; sind(yThe(i)) 0 cosd(yThe(i))]*[1 0 0; 0 cosd(xThe(i)) sind(xThe(i)); 0 -sind(xThe(i)) cosd(xThe(i))]*[xAcc(i) yAcc(i) zAcc(i)]'; %Tracks all rotations
        %acca2 = [cosd(yThe) 0 -sind(yThe); 0 1 0; sind(yThe) 0 cosd(yThe)]*acca1; %Tracks second rotation with y rotation
        %acca3 = [1 0 0; 0 cosd(xThe) sind(xThe); 0 -sind(xThe) cosd(xThe)]*acca2; %Tracks final rotation with x rotation
        xAcca(i) = acca(1);
        yAcca(i) = acca(2);
        zAcca(i) = acca(3);
    end
    
    for i = 1:1:(length(xAcca)-1) %Calculates velocity at each time point
        xVel(i+1) = xVel(i) + 0.5*t*(xAcca(i) + xAcca(i+1));
        yVel(i+1) = yVel(i) + 0.5*t*(yAcca(i) + yAcca(i+1));
        zVel(i+1) = zVel(i) + 0.5*t*(zAcca(i) + zAcca(i+1));
    end
    
    for i = 1:1:(length(xVel)-1) %Calculates distance, angle at each time point
        xDist(i+1) = xDist(i) + 0.5*t*(xVel(i) + xVel(i+1));
        yDist(i+1) = yDist(i) + 0.5*t*(yVel(i) + yVel(i+1));
        zDist(i+1) = zDist(i) + 0.5*t*(zVel(i) + zVel(i+1));
    end
    
    close all %Closes any plots that may be open from previous runs of the code
    
    figure;
    plot(curtime, xDist) %2D plot of distance vs time
    title("xDist vs. time");
    xlabel("Time, t (sec)");
    ylabel("X Distance, d_x (m)");
    
    figure;
    plot(curtime, yDist) %2D plot of distance vs time
    title("yDist vs. time");
    xlabel("Time, t (sec)");
    ylabel("Y Distance, d_y (m)");
    
    figure;
    plot(curtime, zDist) %2D plot of distance vs time
    title("zDist vs. time");
    xlabel("Time, t (sec)");
    ylabel("Z Distance, d_z (m)");
    
    figure;
    plot(curtime, xAcc) %2D plot of acceleration vs time
    title("xAcc vs. time");
    xlabel("Time, t (sec)");
    ylabel("X Acceleration, a_x (m/s^2)");
    
    figure;
    plot(curtime, yAcc) %2D plot of acceleration vs time
    title("yAcc vs. time");
    xlabel("Time, t (sec)");
    ylabel("Y Acceleration, a_y (m/s^2)");
    
    figure;
    plot(curtime, zAcc) %2D plot of acceleration vs time
    title("zAcc vs. time");
    xlabel("Time, t (sec)");
    ylabel("Z Acceleration, a_z (m/s^2)");
    
    figure;
    plot(curtime, xOme) %2D plot of angular velocity vs time
    title("xOme vs. time");
    xlabel("Time, t (sec)");
    ylabel("X Angular Velocity, \theta_x (deg/s)");
    
    figure;
    plot(curtime, yOme) %2D plot of angular velocity vs time
    title("yOme vs. time");
    xlabel("Time, t (sec)");
    ylabel("Y Angular Velocity, \theta_y (deg/s)");
    
    figure;
    plot(curtime, zOme) %2D plot of angular velocity vs time
    title("zOme vs. time");
    xlabel("Time, t (sec)");
    ylabel("Z Angular Velocity, \theta_z (deg/s)");
    
    figure;
    plot3(xDist, yDist, zDist) %3D plot of distance
    title("Trajectory Plot");
    xlabel("X Distance, d_x (m)");
    ylabel("Y Distance, d_y (m)");
    zlabel("Z Distance, d_z (m)");
    
    time = t*(length(xDist)) %total time of flight
    apogee = max(zDist)
    location = [xDist(length(xDist)) yDist(length(yDist)) zDist(length(zDist))] %Final location -- x, y, z coordinate
end
