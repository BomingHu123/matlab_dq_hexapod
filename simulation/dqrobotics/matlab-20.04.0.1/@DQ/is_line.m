% IS_LINE(x) receives a dual quaternion x and returns 1 if it is a line
% (i.e., Re(x) = 0 and norm(x) = 1), and 0 otherwise. 
% See also is_plane, is_pure, is_pure_quaternion, is_quaternion, is_real,
% is_real_number, is_unit

% (C) Copyright 2011-2019 DQ Robotics Developers
% 
% This file is part of DQ Robotics.
% 
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.sourceforge.net
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br

function ret = is_line(x)
    if is_pure(x) && is_unit(x)
        ret = 1;
    else
        ret = 0;
    end
end
