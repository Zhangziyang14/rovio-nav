function [x_k P_k] = lkf(x,z_m,P,F,H,Q,R)
% lkf(x,z_m,P,F,H,Q,R) Runs a linear Kalman filter given the state and
%                      uncertainty x and P at time (k-1), the measurement
%                      data z_m the state transition and measurement model 
%                      transformation matrices F and H, and the state and
%                      model noise covariances Q and R.
%
%                      Returns: [x_k P_k] where x_k is the state at time k
%                               and P_k is the covariance matrix (uncertainty)
%                               at time k.

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

    % calculate transposes ahead of time for efficiency
    F_t = F';
    H_t = H';

    % project state through time and put through measurement model
    x_k = F*x;
    z   = H*x_k;

    % calculate innovation and innovation covariance
    v = zeros(size(z));
    v(1:4) = z_m(1:4) - z(1:4);
    S = H*P*H_t + R;

    % special case due to angle periodicity
    v(5) = subt_angle(z_m(5),z(5));

    % calculate Kalman gain
    K = P*H_t*(S^-1);

    % update the state
    delta    = K*v;
    %x_k = x_k + delta;

    % special case due to angle periodicity
    x_k(1:4) = x_k(1:4) + delta(1:4);
    x_k(5)   = add_angle(x_k(5),delta(5));

    % update the covariance
    P_k = F*P*F_t + Q - F*P*H_t*(S^-1)*H*P*F_t;
