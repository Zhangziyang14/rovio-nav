function c = subt_angle(a,b)
% subt_angle(a,b) Calculates (a-b) where a and b are both angles.
%
%                 Returns: The atan2 difference of a and b (NOTE: atan2
%                          has a discontinuity at pi/2).

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

    qa = [cos(a) sin(a)];
    qb = [cos(b) sin(b)];

    c = atan2(qa(2)*qb(1)-qb(2)*qa(1),qa(1)*qb(1)+qa(2)*qb(2));
