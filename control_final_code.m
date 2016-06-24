clc; clear all; close all;
mprim_endx = 0; mprim_endy = 0;
%% This section plots the grid. While we redraw the grid each time while plotting
% the position of the robot, this section draws the grid, objects, and the
% robot the first time

figure(1)
% gridlines ---------------------------
hold on
y_axis=[0:1:10]; % Y grid defined by user[start:spaces:end]
x_axis=[0:1:10]; % X grid defined by user[start:spaces:end]
for i=1:length(x_axis)
    plot([x_axis(i) x_axis(i)],[y_axis(1) y_axis(end)],'k:') %Y grid lines
    hold on
end
for i=1:length(y_axis)
    plot([x_axis(1) x_axis(end)],[y_axis(i) y_axis(i)],'k:') %X grid lines
    hold on
end

% Plotting objects
object1_x = 5.5; object1_y = 4.5;
object2_x = 5.5; object2_y = 5.5;
plot(object1_x, object1_y,'rs');
plot(object2_x, object2_y,'rs');

%Plotting initial position of robot
x_initialization = 1.5; y_initialization = 5.5;
x_robot_current = x_initialization; y_robot_current = y_initialization;
plot(x_robot_current, y_robot_current, 'ro')

%% This line of code waits for user input. Currently to show the working of
%the controller we are just taking one direction as input
keyPressed = getkey();
while keyPressed == 'w'
    %% Calculating the x,y and theta for the motion primitive
    %This section of code is taken from the SBPL MATLAB library for
    %generating motion primitives. www.sbpl.net
    %Necessary changes are made to suit our needs. We only generate 5
    %motion primitives per angle, and also here we only use 0 degrees angle
    resolution = 1;
    numberofangles = 16; %preferably a power of 2, definitely multiple of 8
    numberofprimsperangle = 5;
    angleind = 1;
    
    %multipliers (multiplier is used as costmult*cost), again since our aim
    %is to show the working of controller and not a path planning algorithm
    %we assign a unit cost to all the motions
    forwardcostmult = 1;
    backwardcostmult = 1;
    forwardandturncostmult = 1;
    
    
    %0 degreees; Angles are positive counterclockwise
    basemprimendpts0_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult
    %0 theta change
    basemprimendpts0_c(1,:) = [2 0 0 forwardcostmult];
    basemprimendpts0_c(2,:) = [-1 0 0 backwardcostmult];
    basemprimendpts0_c(5,:) = [1 0 0 forwardcostmult];
    %1/16 theta change
    basemprimendpts0_c(3,:) = [2 1 1 forwardandturncostmult];
    basemprimendpts0_c(4,:) = [2 -1 -1 forwardandturncostmult];
    
    for primind = 1:numberofprimsperangle
        storage_index = 1;
        %current angle
        currentangle = (angleind-1)*2*pi/numberofangles;
        currentangle_36000int = round((angleind-1)*36000/numberofangles);
        
        basemprimendpts_c = basemprimendpts0_c(primind,:) + [x_robot_current, y_robot_current, 0, 0];
        angle = currentangle;
        fprintf(1, '90\n');
        
        baseendpose_c = basemprimendpts_c(1:3);
        additionalactioncostmult = basemprimendpts_c(4);
        endxc = (baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));
        endyc = (baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
        
        endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
        endpose_c = [endxc endyc endtheta_c];
        
        %generate intermediate poses (remember they are w.r.t 0,0 (and not
        %centers of the cells)
        numofsamples = 50;
        intermcells_m = zeros(numofsamples,3);
        
        startpt = [x_robot_current y_robot_current currentangle];
        endpt = [endpose_c(1)*resolution endpose_c(2)*resolution ...
            rem(angleind - 1 + baseendpose_c(3), numberofangles)*2*pi/numberofangles];
        if primind == 1
            mprim_endx = endpt(1); 
            mprim_endy = endpt(1);
        end
        intermcells_m = zeros(numofsamples,3);
        if ((endxc == 0 & endyc == 0) || baseendpose_c(3) == 0) %turn in place or move forward
            for iind = 1:numofsamples
                intermcells_m(iind,:) = [startpt(1) + (endpt(1) - startpt(1))*(iind-1)/(numofsamples-1) ...
                    startpt(2) + (endpt(2) - startpt(2))*(iind-1)/(numofsamples-1) ...
                    0];
                rotation_angle = (baseendpose_c(3) ) * (2*pi/numberofangles);
                intermcells_m(iind,3) = rem(startpt(3) + (rotation_angle)*(iind-1)/(numofsamples-1), 2*pi);
            end;
        else %unicycle-based move forward or backward
            R = [cos(startpt(3)) sin(endpt(3)) - sin(startpt(3));
                sin(startpt(3)) -(cos(endpt(3)) - cos(startpt(3)))];
            S = pinv(R)*[endpt(1) - startpt(1); endpt(2) - startpt(2)];
            l = S(1);
            tvoverrv = S(2);
            rv = (baseendpose_c(3)*2*pi/numberofangles + l/tvoverrv);
            tv = tvoverrv*rv;
            
            if ((l < 0 & tv > 0) | (l > 0 & tv < 0))
                fprintf(1, 'WARNING: l = %d < 0 -> bad action start/end points\n', l);
                l = 0;
            end;
            for iind = 1:numofsamples
                dt = (iind-1)/(numofsamples-1);
                
                if(abs(dt*tv) < abs(l))
                    intermcells_m(iind,:) = [startpt(1) + dt*tv*cos(startpt(3)) ...
                        startpt(2) + dt*tv*sin(startpt(3)) ...
                        startpt(3)];
                    
                else
                    dtheta = rv*(dt - l/tv) + startpt(3);
                    intermcells_m(iind,:) = [startpt(1) + l*cos(startpt(3)) + tvoverrv*(sin(dtheta) - sin(startpt(3))) ...
                        startpt(2) + l*sin(startpt(3)) - tvoverrv*(cos(dtheta) - cos(startpt(3))) ...
                        dtheta];
                    
                end;
            end;
            %correct
            errorxy = [endpt(1) - intermcells_m(numofsamples,1) ...
                endpt(2) - intermcells_m(numofsamples,2)];
            fprintf(1, 'l=%f errx=%f erry=%f\n', l, errorxy(1), errorxy(2));
            interpfactor = [0:1/(numofsamples-1):1];
            intermcells_m(:,1) = intermcells_m(:,1) + errorxy(1)*interpfactor';
            intermcells_m(:,2) = intermcells_m(:,2) + errorxy(2)*interpfactor';
        end;
        
        plot(intermcells_m(:,1), intermcells_m(  :,2));
        axis([0 10 0 10]);
        
        %This stores the co-ordinates of the trajectories
        xcoordinates(primind, :) = intermcells_m(:,1);
        ycoordinates(primind, :) = intermcells_m(:,2);
        theta(primind, :) = intermcells_m(:,3);
        
        storage_index = storage_index + 1;
        hold on;
    end
    
    
    
    %% Controller - After generating the motion primitives, this section of code
    % is the controller which is used to track and follow a motion
    % primitive, the controller has been developed using the method of
    % Approximate linearization mentioned from page number 503
    % to 507 in the book "Robotics Modelling, Planning and Control" by Bruno 
    % Siciliano 
    
    %% This section checks if the the end point of the motion primitive collides
    % with the object
    if (mprim_endx == object2_x) && (mprim_endy == object2_y)
        mprim_used = 3;
    else
        mprim_used = 5;
    end
    
    %% Initial conditions
    x_0 = xcoordinates(mprim_used,1);
    y_0 = ycoordinates(mprim_used,1);
    th_0 = theta(mprim_used,1);
    
    dc = 0.2;%damping coefficient
    a = 1; %natural frequency
    tstep = 1;
    
    %% xc, yc and thc denote the current x,y and z co-ordinates
    xc = x_0;
    yc = y_0;
    thc = th_0;
    e_dynamics = [];
    
    for t = 1:1:49
        %% Since we are drawing the image in each loop again, this code draws
        % the grid
        hold on
        y_axis=[0:1:10]; % Y grid defined by user[start:spaces:end]
        x_axis=[0:1:10]; % X grid defined by user[start:spaces:end]
        for i=1:length(x_axis)
            plot([x_axis(i) x_axis(i)],[y_axis(1) y_axis(end)],'k:') %Y grid lines
            hold on
        end
        for i=1:length(y_axis)
            plot([x_axis(1) x_axis(end)],[y_axis(i) y_axis(i)],'k:') %X grid lines
            hold on
        end
        %% This section draws the motion primitives
        plot(xcoordinates(1,:), ycoordinates(1,:));
        plot(xcoordinates(2,:), ycoordinates(2,:));
        plot(xcoordinates(3,:), ycoordinates(3,:));%intermcells_m(:,1), intermcells_m(:,2));
        plot(xcoordinates(4,:), ycoordinates(4,:));
        
        %% This section draws the objects
        plot(object1_x, object1_y,'rs');
        plot(object2_x, object2_y,'rs');
        axis([0 10 0 10]);
        
        %% This line draws a circle at the current position of the robot, the
        % robot is the circle in the red color
        plot(xc,yc,'ro');
        %% This line draws a circle to show the heading of the robot(thc) at
        % the current position of the robot. This circle is drawn in blue
        % color
        plot(xc + 0.1*cos(thc), yc + 0.1*sin(thc),'bo');
        pause(0.25);
        clf;
        %% Desired position of the robot
        xd = xcoordinates(mprim_used,t+1);
        yd = ycoordinates(mprim_used,t+1);
        thd = theta(mprim_used,t+1);
        
        %% Error matrix
        e = [cos(thc), sin(thc),0; -sin(thc),cos(thc),0;0,0,1]*[xd - xc; yd - yc; thd - thc];
        
        %% Desired translational and angular velocity
        vd = (xd - xcoordinates(mprim_used,t))/tstep; %translational velocity
        wd = (thd - theta(mprim_used,t))/tstep; %angular velocity
        %% The gains k1, k2 and k3
        k1 = 2*dc*a; k3 = 2*dc*a; k2 = (a^2 - wd^2)/vd;
        %% Inputs u1 and u2
        u1 = -k1*e(1,1);
        u2 = -k2*e(2,1) - k3*e(3,1);
        %% The current translational and angular veloctiy of robot
        vc = vd*cos(e(3,1)) - u1;
        wc = wd - u2;
        
        %% The current x and y coordinate, and orientation of robot
        disturbance = 0;
        thc = thc + wc*tstep;
        xc = xc + vc*cos(thc)*tstep + disturbance;
        yc = yc + vc*sin(thc)*tstep + disturbance;
        
        %% This section stores the desired and current values, the errors 
        % which are used for plotting the graphs
        if mprim_used == 3
            e_d = [0, wd, 0; -wd, 0, vd; 0,0,0]*e + [1,0;0,0;0,1]*[u1;u2];
            %e_dynamics = cat(3, e_dynamics, e_d );
            error_x(t) = e_d(1);
            error_y(t) = e_d(2);
            error_th(t) = e_d(3);
            xc_coord(t) = xc;
            yc_coord(t) = yc;
            thc_coord(t) = thc;
            error_direct_x(t) = e(1);
            error_direct_y(t) = e(2);
            error_direct_th(t) = e(3);
            xd_plot(t) = xd;
            yd_plot(t) = yd;
            thd_plot(t) = thd;
        end
    end
    x_robot_current = x_robot_current + 1;
    if mprim_used == 3
        y_robot_current = y_robot_current + 1;
    end
    keyPressed = getkey();
    
end






