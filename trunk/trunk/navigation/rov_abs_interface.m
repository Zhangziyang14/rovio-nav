function rov_abs_interface(addr,locs,aut,n)
% rov_abs_interface   Provides a live interface for the Rovio's absolute
%   (addr,locs,aut,n) positioning system. addr indicates the address of
%                     the Rovio, locs is a nx2 matrix of n points to reach,
%                     which will be performed autonomously if aut is set to
%                     true. n determines the noisiness of the area; this is
%                     best kept between 0 and 1.
%
%                     In the interface, the red objects are raw data.
%                     Blue objects are filtered data.
%
%                     Returns: nothing.

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
 
    % get rid of all rotation variables
    clear abs_est;
    
    % rovio address
    r = rovio(addr);

    % movement info
    dt      = 0.325;
    dims    = size(locs);
    num_pts = dims(1);
    cur_pt  = 1;
    goal_reached = false;

    % plotting info
    lims = [-300 300];
    f = figure('OuterPosition',[0 0 800 400],...
               'Units','pixels',...
               'Name','Rovio Absolute Position Information',...
               'NumberTitle','off');
           
    while true && ishandle(f)
        tstart = tic;
        
        [x_est z_meas] = abs_est(r,n);
        
        % if auton mode is on, move towards the goal location
        if aut ~= false && goal_reached == false
            goal_reached = move_towards(r,x_est,locs(cur_pt,:));
        end
        
        % if goal has been reached, move to the next goal location
        if goal_reached == true && cur_pt < num_pts
            goal_reached = false;
            cur_pt = cur_pt + 1;
        end
        
        clf(f);
        
        subplot(1,2,1);
        hold on;
        axis([lims lims]);
        grid on;
        plot(z_meas(1),z_meas(2),'ro',x_est(1),x_est(2),'bo');
        title('Rovio absolute position'); 
        hold off;
        
        subplot(1,2,2);
        hold on;
        axis([-1 1 -1 1]);
        grid on;
        line('XData',[0 cos(z_meas(5))],...
             'YData',[0 sin(z_meas(5))],...
             'Color','r');
        line('XData',[0 cos(x_est(5))],...
             'YData',[0 sin(x_est(5))],...
             'Color','b');
        title('Rovio absolute angle'); 
        hold off;
        drawnow;
        
        % wait until dt time has passed
        elapsed = toc(tstart);
        if elapsed > dt
            fprintf('Warning: lag detected!\n');
        end
        
        while elapsed < dt
            elapsed = toc(tstart);
        end
    end
end
