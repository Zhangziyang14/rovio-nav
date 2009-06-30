function [x_est z_meas] = abs_est(r,n)
% abs_est(r,n) Estimates the position of the Rovio object r relative to its
%              starting beacon with a noise level of n (best between 0 
%              and 1). This is roughly as close as one can get to an 
%              "absolute" position, but it is extraordinarily useful.
%            
%              Returns: [x_est z_meas] where x_est is the estimate of the
%                       state at time k and z_meas it the measurement of
%                       the absolute position at time k. Both state and
%                       measurement are expressed as [p_x p_y v_x v_y t]'
%                       where the variables correspond to:
%                         p_x: Position along x-axis
%                         p_y: Position along y-axis
%                         v_x: Velocity along x-axis
%                         v_y: Velocity along y-axis
%                           t: Theta, counterclockwise angle from x-axis

%{
Copyright 2009 John Schaeffer

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
%}

    % initialize persistent variables
    persistent initialized...
               rotations...
               p_shifts...
               angles...
               rot_stored...
               prev_est...
               cur_room...
               F H P Q R...
               scaling
    
    % initialization stage
    if initialized
        initialized = true;
    else 
        % rotation info
        rotations  = zeros(2,2,10); % 2x2x10 matrix: 2x2 rotation matrix with
                                    % 10 copies for 10 (max) beacons

        p_shifts   = zeros(2,1,10); % 2x1x9 matrix: 2x1 shift vector with 10
                                    % copies for 10 (max) beacons

        angles     = zeros(10,1);   % 10x1 vector: 10 angle differences, 1 for
                                    % each beacon

        rot_stored = zeros(10,1);   % set to 1 if rotation matrix is stored
        
        % kalman filter info
        scaling  = 100/6000; % scaling factor (feel free to tweak this)
        prev_est = zeros(5,1); % state model: [pos_x pos_y vel_x vel_y theta]'

        P = eye(5);                             % uncertainty/noise matrices
        Q = diag([0.01 0.01 0.001 0.001 0.01]);
        R = diag([0.1 0.1 0.1 0.1 0.005]*(n + 0.1));

        F = [1 0 1 0 0;  % transition model: p_x + v_x
             0 1 0 1 0;  %                   p_y + v_y
             0 0 1 0 0;  %                   v_x
             0 0 0 1 0;  %                   v_y
             0 0 0 0 1]; %                   t

        H = eye(5);      % measurement model is the same as the state
                                  
        [y x theta cur_room] = r.get_Report();
        
        cur_room = cur_room + 1;          % fix due to base room being 0
    
        rot_stored(cur_room) = 1;         % starting room is absolute origin
        rotations(:,:,cur_room) = eye(2); % no rotation or shift
        
        initialized = true;
    end
    
    [y x theta cur_room] = r.get_Report();
    y = y*scaling;
    x = -x*scaling; % rovio axes are inverted

    cur_room = cur_room + 1; % rovio counts rooms from 0-9, adjusting

    [xdelta ydelta] = r.get_delta();

    z_meas = [x y xdelta ydelta theta]';

    % start room check
    if rot_stored(cur_room) == 0 % entered a new beacon area

        % calculate the angle of rotation
        d_ang = subt_angle(prev_est(5),z_meas(5));

        % calculate the rotation matrix here
        rotations(:,:,cur_room) = [cos(d_ang), -sin(d_ang);
                                   sin(d_ang),  cos(d_ang)];

        % calculate the distance between origins here
        p_shifts(:,cur_room) = prev_est(1:2)...
                               - rotations(:,:,cur_room)*z_meas(1:2);

        % calculate the angle between origins
        angles(cur_room) = d_ang;

        % matrix is stored!
        rot_stored(cur_room) = 1;

        % uncomment to print the new room information
        %{
        fprintf('New room %d found!\n',cur_room);
        fprintf('Offset: [%.4f, %.4f]\n',...
                p_shifts(1,cur_room),p_shifts(2,cur_room));
        fprintf('Angle: %.4f\n',angles(cur_room));
        %}
    end

    % rotate and shift the measurement position and velocities
    z_meas(1:2) = rotations(:,:,cur_room)*z_meas(1:2)...
                  + p_shifts(:,:,cur_room);
    z_meas(3:4) = rotations(:,:,cur_room)*z_meas(3:4);

    % set the angle
    z_meas(5) = add_angle(z_meas(5),angles(cur_room));

    % filter the state
    [x_est P] = lkf(prev_est,z_meas,P,F,H,Q,R);

    % capture the current state for rotation purposes
    prev_est  = x_est;
    
end
