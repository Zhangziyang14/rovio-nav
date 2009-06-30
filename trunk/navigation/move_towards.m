function reached = move_towards(r,x,p)
% move_towards(r,x,p) Moves the Rovio at connection r towards point p,
%                     assuming its current state (see 'help abs_est') is 
%                     described in x.
%                     
%                     Returns: true if the goal is reached, false otherwise.

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

    dist = norm([p(1)-x(1), p(2)-x(2)]); % magnitude of the distance vector
        
    angle = atan2(p(2)-x(2), p(1)-x(1)); % angle from the x-axis

    err_t = subt_angle(x(5),angle);

    range_t = 40*(pi/180); % 40 degree error (steep, but necessary)
    range_p = 20;          % we need to get within 20 clicks of the target

    mag = 10;              % tweak this to change turning speed

    if dist <= range_p   % we've reached the goal!
        reached = true;
        return;
    else                 % we haven't reached the goal...
        reached = false;
    end

    if err_t < -range_t     % we need to turn left to get back on track
        r.turnLeft(1,mag);
    elseif err_t > range_t  % we need to turn right to get back on track
        r.turnRight(1,mag);
    else                    % we didn't need to turn, so let's move
        r.move(1,10);
    end
