% Define the control objetives to be used with the DQ_KinematicController class.
%
% The following objectives are currently implemented:
%   Distance,
%   DistanceToPlane,
%   Line,
%   None,
%   Plane,
%   Pose,
%   Rotation,
%   Translation.
%
% See also DQ_KinematicController.

% (C) Copyright 2011-2019 DQ Robotics Developers
%
% This file is part of DQ Robotics.
%
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as
%     published by the Free Software Foundation, either version 3 of the
%     License, or (at your option) any later version.
%
%     DQ Robotics is distributed in the hope that it will be useful, but
%     WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%     Lesser General Public License for more details.
%
%     You should have received a copy of the GNU Lesser General Public
%     License along with DQ Robotics.  If not, see
%     <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: dqrobotics.github.io
%
% Contributors to this file:
%     Bruno Vihena Adorno - adorno@ufmg.br
classdef ControlObjective
    enumeration 
        Distance, DistanceToPlane, Line, None, Plane, Pose, Rotation,...
            Translation, HexapodTask, HexapodAbs, HexapodRel,HexapodTranslationTask,
    end
end
