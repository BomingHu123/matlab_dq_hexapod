
classdef DQ_MultilegRobot_Leg < DQ_SerialManipulator
    methods
        function obj = DQ_MultilegRobot_Leg(A,convention)
            obj = obj@DQ_SerialManipulator(A(1:4,:),convention);
        end
        function plot(robot,q,varargin)
            % plot(robot,q,options) plots the robot of type DQ_kinematics.
            % q is the vector of joint configurations
            % options is an optional argument that has variable size and accept any
            % number of the following pairs:
            %
            %  'workspace', W          size of robot 3D workspace, where
            %                          W = [xmn, xmx ymn ymx zmn zmx]
            %  'cylinder', C           color for joint cylinders, C=[r g b]
            %  'scale', scale          annotation scale factor
            %  'base'|'nobase'         controls display of base plane
            %  'wrist'|'nowrist'       controls display of wrist
            %  'name'|'noname'         display the robot's name
            %  'xyz'|'noa'             wrist axis label
            %  'joints'|'nojoints'     controls display of joints
            %
            % The graphical robot object holds a copy of the robot object and
            % the graphical element is tagged with the robot's name (.name property).
            %
            % 1) Figure behavior:
            %
            % If no robot of this name is currently displayed then a robot will
            % be drawn in the current figure.  If hold is enabled (hold on) then the
            % robot will be added to the current figure.
            %
            % If the robot already exists then that graphical model will be found
            % and moved.
            %
            % 2) Multiple views of the same robot:
            %
            % If one or more plots of this robot already exist then these will all
            % be moved according to the argument 'q'.  All robots in all windows with
            % the same name will be moved.
            %
            % NOTE: Since each kinematic robot stores just one graphical handle,
            % if we want to plot the same robot in different views, we must declare
            % different robots with the same name. Otherwise, if just one robot is declared,
            % but plotted in different windows/views, the kinematic robot will store
            % the handle of the last view only. Therefore, only the last view will be
            % updated
            %
            % 3) Multiple robots in the same figure:
            %
            % Multiple robots (i.e., with different names) can be displayed in the same
            % plot, by using "hold on" before calls to plot(robot).
            %
            % 4) Graphical robot state:
            %
            % The configuration of the robot as displayed is stored in the DQ_kinematics
            % object and can be accessed by the read only object property 'q'.
            %
            % 5) Graphical annotations and options:
            %
            % The robot is displayed as a basic stick figure robot with annotations
            % such as:
            % - XYZ wrist axes and labels,
            % - joint cylinders,
            % which are controlled by options.
            %
            % The size of the annotations is determined using a simple heuristic from
            % the workspace dimensions.  This dimension can be changed by setting the
            % multiplicative scale factor using the 'scale' option.
            has_options = 0;
            if nargin < 2
                fprintf(['\nUsage: plot(robot, q,[options])'...
                    '\ntype ''help DQ_kinematics/plot'' for more information\n']);
                return;
            elseif nargin >= 3
                has_options = 1;
            end
            
            if ~isvector(q)
                error('The first argument must be the vector of joint configurations.');
            end
            
            % The joint configuration vector must be a row vector
            if ~isrow(q)
                q = q';
            end
            
            if has_options
                dq_kinematics_plot(robot,q, varargin{:});
            else
                dq_kinematics_plot(robot,q);
            end
        end
    end
end

function dq_kinematics_plot(robot, q, varargin)
    % process options
    if (nargin > 2) && isstruct(varargin{1})
        % options is a struct. 
        opt = varargin{1};
    else
        % options is a list of options; we need to transform it to a struct
        opt = plot_options(robot, varargin);
    end

    n = robot.n_links;

    if length(q) ~= n
        error('Incorrect number of joints. The correct number is %d', n);
    end

    % get handles of all existing robot with the same name
    graphic_robot_handle = findobj('Tag', robot.name);
    
    % Condition to verify that no robot with this name exists
%     condition1 = isempty(graphic_robot_handle) || isempty(get(gcf, 'Children'));
%     % Condition to verify if hold is on and no robot of this name is in the 
%     % current axes
%     condition2 = ishold && isempty(findobj(gca, 'Tag', robot.name));
    condition1 = 1;
    condition2 = 1;
    if condition1 || condition2
        % no robot with this name exists
        h = create_new_robot(robot, opt);
        % save the handle in the robot object and attach it to the robot 
        % as user data. This way, the data will be available to the
        % update_robot() function.
        robot.handle = h;
        set(h.robot, 'Tag', robot.name);
        set(h.robot, 'UserData', robot);
        graphic_robot_handle = h.robot;        
    end

    % Update all robots with the same name. This is very useful when we
    % want to visualize the same robot from different points of view
    % Each element of graphic_robot_handle has a graphic line robot
    for r = graphic_robot_handle'    
        % Inside the graphic line robot, we store the 'real' robot. We use
        % it in order to plot the robot.
        rr = get(r, 'UserData');
        update_robot(rr, q);
        % save the joint angles away in all the graphical robots with the
        % same name
        rr.q = q;
    end
end

% h = create_new_robot(robot, opt) uses data from robot object and options
% to create a graphical robot.
%
% Returns a structure of handles to graphical objects.
%
% If current figure is empty, draw robot in it
% If current figure has hold on, add robot to it
% Otherwise, create new figure and draw robot in it.
%   
% The handle is composed of the following fields:
% h.mag             Robot scale
% h.robot           the line segment that represents the robot
% h.x               the line segment that represents the end-effector x-axis
% h.y               the line segment that represents the end-effector y-axis
% h.z               the line segment that represents the end-effector z-axis
% h.xt              text for the end-effector x-axis
% h.yt              text for the end-effector y-axis
% h.zt              text for the end-effector z-axis
% h.joint(i)        the i-th joint (cylinder for revolute joints)
% h.base_handle     the robot base plane 
% h.name_handle     the robot name
function h = create_new_robot(robot, opt)
    h.mag = opt.mag;

    if ~ishold
        % if current figure has hold on, then draw robot here
        % otherwise, create a new figure
        axis(opt.workspace);
    end
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    set(gca, 'SortMethod', 'depth');
    grid on

    % Draw a small plane representing the robot base
    if opt.base
        plane = robot.base_frame.'*DQ.k*robot.base_frame';
        % Since the plane is infinite, the DQ.plot function draws the part
        % closest to the origin of the reference frame. We first
        % consider the plane that passes through the origin and is aligned with 
        % the one that supports the base
        base_handle = plot(plane.P,'plane',opt.mag,'color','k');
        % We then translate the 'visible' plane to the base frame
        base_translation = vec3(translation(robot.base_frame));
        plane_vertices = get(base_handle, 'Vertices');
        for i = 1:3
            plane_vertices(:,i) = plane_vertices(:,i) + base_translation(i);
        end        
        set(base_handle, 'Vertices', plane_vertices);
        h.base_handle = base_handle;
    end
     
    % Write the robot name.
    if opt.name        
        b = vec3(translation(robot.base_frame));
        h.name_handle = text(b(1), b(2) - opt.mag, b(3), [' ' robot.name],...
            'FontAngle', 'italic','FontWeight', 'bold');
    end
    
    % create a line that we will subsequently modify using the function 
    % update_robot(). 
    h.robot = line(robot.lineopt{:});
    
    % create end-effector frame
    if opt.wrist   
        h.x = line('xdata', [0;0], ...
            'ydata', [0;0], ...
            'zdata', [0;0], ...
            'color', 'red');
        h.y = line('xdata', [0;0], ...
            'ydata', [0;0], ...
            'zdata', [0;0], ...
            'color', 'green');
        h.z = line('xdata', [0;0], ...
            'ydata', [0;0], ...
            'zdata', [0;0], ...
            'color', 'blue');
        h.xt = text(0, 0, opt.wristlabel(1), 'FontWeight', 'bold',...
            'HorizontalAlignment', 'Center');
        h.yt = text(0, 0, opt.wristlabel(2), 'FontWeight', 'bold',...
            'HorizontalAlignment', 'Center');
        h.zt = text(0, 0, opt.wristlabel(3), 'FontWeight', 'bold',...
            'HorizontalAlignment', 'Center');

    end

    % Display cylinders (revolute each joint).
    for i = 1:robot.n_links
        if opt.joints
            %TODO: implement prismatic joints
            N = 8;
            % define the vertices of the unit cylinder with radius opt.mag/4
            % the z coordinates of the bottom and top faces are 0 and 1,
            % respectively. Furthermore, xc, yc, and zc have 2 rows: the
            % first corresponds to the bottom coordinates and the second to
            % the top coordinates.
            [xc,yc,zc] = cylinder(opt.mag/4, N);            
            
            % Scale the cylinder
            % when zc == 0, make it -opt.mag/2, except for the first joint
            if i~= 1
                zc(zc==0) = -opt.mag/2;
            end
            % when zc == 1, make it opt.mag/2
            zc(zc==1) = opt.mag/2;

            % Create vertex color data. Each vertex receive a RGB triplet.
            % Red value is stored in cdata(:,:,1), green in cdata(:,:,2)
            % and blue in cdata
            cdata = zeros(size(xc,1),size(xc,2),3);
            for j=1:3
                cdata(:,:,j) = opt.cylinder(j);
            end
            % render the surface
            h.joint(i) = surface(xc,yc,zc,cdata);
          
            % set the surfaces to be smooth and translucent
            set(h.joint(i), 'FaceColor', 'interp');
            set(h.joint(i), 'EdgeColor', 'none');
            set(h.joint(i), 'FaceAlpha', 0.7);

            % build a matrix of coordinates so we
            % can transform the cylinder in update_robot()
            % and hang it off the cylinder
            xyz = [xc(:)'; yc(:)'; zc(:)'; ones(1,2*N+2)];
            
            set(h.joint(i), 'UserData', xyz);
        end
    end
end


% update_robot(robot, q) moves an existing graphical robot to the configuration
% specified by the joint coordinates q. The parameter 'robot' is the
% kinematic robot, and graphics are defined by the handle structure robot.handle, 
% which stores the 'graphical robot' as robot.handle.robot.
function update_robot(robot, q)
    n = robot.n_links;
    
    % Get the handle to the graphical robot. Since each kinematic robot
    % stores just one graphical handle, if we want to plot the same robot
    % in different views, we must declare different robots with the same
    % name. Otherwise, if just one robot is declared, but plotted in
    % different windows/views, the kinematic robot will store the handle of
    % the last view only.
    h = robot.handle;
    mag = h.mag;
    base = vec3(translation(robot.base_frame));
    
    % Initialize the vector containing the origin of each frame along the
    % kinematic chain
    x = zeros(n+1,1);
    y = zeros(n+1,1);
    z = zeros(n+1,1);
    % The first element corresponds to the origin of the base frame
    x(1) = base(1);
    y(1) = base(2);
    z(1) = base(3);

    % compute the link transforms, and record the origin of each frame
    % for the graphics update.    
    for j=1:n    
        t = vec3(translation(robot.base_frame*robot.raw_fkm(q,j)));
        x(j+1) = t(1);    
        y(j+1) = t(2);
        z(j+1) = t(3);
    end
    
    % Update the coordinates of each frame along the kinematic chain. This 
    % updates the line drawing that represents the robot kinematic chain.
    set(h.robot,'xdata', x, 'ydata', y, 'zdata', z);
   
    % display the joints as cylinders
    if isfield(h, 'joint')       
        for j=1:n
            % get coordinate data from the cylinder. The corresponding UserData 
            % is never updated. Therefore, we must translate and rotate
            % each cylinder according to the new frames along the kinematic
            % chain
            xyz = get(h.joint(j), 'UserData');
            
            %The joints are located at the beginning of each link
            fkm_j = robot.raw_fkm(q,j-1);
            
            for k = 1:size(xyz,2)
                % 1 + DQ.E*(1/2)*p, where p = xyz(1:3,k);
                cylinder_vertex = DQ([1;0;0;0;0;0.5*xyz(1:3,k)]);
                xyz(1:3,k) = vec3(translation(robot.base_frame*fkm_j*cylinder_vertex));
            end
            
            % Now that all cylinder vertices are transformed, update the
            % cylinder drawing. Transform xc, yc, and zc to matrices of
            % two rows and ncols columns (i.e., half the number of vertices
            % as each column stores the bottom and top vertices).
            ncols = size(xyz,2)/2;
            xc = reshape(xyz(1,:), 2, ncols);
            yc = reshape(xyz(2,:), 2, ncols);
            zc = reshape(xyz(3,:), 2, ncols);
            
            set(h.joint(j), 'Xdata', xc, 'Ydata', yc, 'Zdata', zc);
        end
    end

   
    % display the wrist axes and labels   
    % compute the wrist axes, based on final link transformation plus the
    % tool transformation.
    if isfield(h, 'x')
        % get the end-effector pose (considering the final transformation given
        % by set_end_effector()
        t = robot.base_frame*robot.raw_fkm(q)*robot.effector;
        t1 = vec3(translation(t));        
        
        % The following transformations use the Hamilton operators to
        % improve performance as Matlab is optimized for matrix operations.
        
        % Recall that, given a rotation t.P, the point transformation of any
        % point p is given by t.P*p*t.P'. In terms of Hamilton operators,
        % the transformation is given by 
        % vec4(t.P*p*t.P') = hamiplus4(t.P)*haminus4(t.P')*vec4(p).
        
        % H(1,2:4) is always 0.
        H = hamiplus4(t.P)*haminus4(t.P');
        
        % We can simplify the calculations as follows
        xv = t1 + H(2:4,2)*mag; % p = [0; mag; 0; 0]
        yv = t1 + H(2:4,3)*mag; % p = [0; 0; mag; 0]
        zv = t1 + H(2:4,4)*mag; % p = [0; 0; 0; mag]        
        
        % update the wrist axes       
        set(h.x,'xdata',[t1(1) xv(1)], 'ydata', [t1(2) xv(2)], ...
            'zdata', [t1(3) xv(3)]);
        set(h.y,'xdata',[t1(1) yv(1)], 'ydata', [t1(2) yv(2)], ...
             'zdata', [t1(3) yv(3)]);
        set(h.z,'xdata',[t1(1) zv(1)], 'ydata', [t1(2) zv(2)], ...
             'zdata', [t1(3) zv(3)]);
         
        % update the axes' name positions 
        set(h.xt, 'Position', xv);
        set(h.yt, 'Position', yv);
        set(h.zt, 'Position', zv);
    end
end


% o = plot_options(robot, options) returns an options structure. 
% 'robot' is the kinematic robot and 'options' is an array cell with the plot
% options.
function o = plot_options(robot, optin)
    % process a cell array of options and return a struct   
    % define all possible options and their default values
    o.joints = true; % Plot the joints
    o.wrist = true; % Plot the end-effector coordinates   
    o.base = true; % Plot a small plane perpendicular to the first joint
    o.wristlabel = 'xyz'; % Axes names. Another option is 'nsa'
    o.scale = 1; % Scale the drawing
    o.name = true; % Write the robot name
    o.cylinder = [0 0 0.7]; % The joint colors   
    o.workspace = [];  % Define the robot workspace. If not defined, a 
                       % heuristic is used to determine the robot workspace
                       % based on its size.

    % Plot options can be stored in the robot as a cell array. If this
    % information is available, we use it together with the options passed to 
    % the plot function.
    options = [robot.plotopt optin];
    
    % parse the options
    if ~isempty(options)
        [o,args] = parse_options(o, options);        
        if ~isempty(args)
            error(['Unknown options: ', args{:}]);
        end
    end

    % NOTE: The calculations below happen all the time. Maybe we should
    % store it in the graphical robot in order to do it just once.
    
    % simple heuristic to figure the maximum reach of the robot
    if isempty(o.workspace)
        reach = 0;
        for i=1:robot.n_links
            % Since the maximum reaching distance are given by the link offset 
            % and link length, we add them.
            reach = reach + abs(robot.a(i)) + abs(robot.d(i));
        end
        o.workspace = [-reach reach -reach reach -reach reach];      
    else
        reach = min(abs(o.workspace));
    end
    % The size of the joints will depend on the size of the workspace. This
    % can be adjusted by using the parameter 'scale'
    o.mag = o.scale * reach/15;
end

% [opt, others] = parse_options(default, options) parses the cell array inside
% 'options' and returns the corresponding structure 'opt'. The default
% parameters are given by 'default'.
%
% The software pattern is:
%       opt.foo = true;
%       opt.bar = false;
%       opt.blah = [];
%
% Optional arguments to the function behave as follows:
%   'foo'           sets opt.foo <- true
%   'nofoo'         sets opt.foo <- false
%   'blah', 3       sets opt.blah <- 3
%
% and can be given in any combination.
%
% NOTE:
% 1) The enumerator names must be distinct from the field names.
% 2) Only one value can be assigned to a field, if multiple values
%    are required they must be converted to a cell array.
%
% The allowable options are specified by the names of the fields in the
% structure opt.  By default if an option is given that is not a field of 
% opt an error is declared.
function [opt,others] = parse_options(default, options)

    arglist = {};

    argc = 1;
    opt = default;

    while argc <= length(options)
        current_option = options{argc};
        assigned = false;

        if ischar(current_option)
            % does the current_option match a field in the opt structure?            
            if isfield(opt, current_option)
                % If yes, then val = opt.(current_option)
                val = getfield(opt, current_option);
                
                %  In case the parameter is something like 'base', the 
                %  corresponding options field must be true. Therefore,
                %  opt.base = true
                if islogical(val)                   
                    opt = setfield(opt, current_option, true);
                else
                    % otherwise grab its value from the next arg
                    % opt.(current_option) = options{argc + 1}
                    opt = setfield(opt, current_option, options{argc+1});
                    argc = argc+1;
                end
                assigned = true;
            % The current option is a string, but does not correspond to an 
            % options field. We verify if the first two letters is 'no' and
            % the remainder of the string corresponds to an options field.
            % In this case, it means that we must assign 'false'to the
            % corresponding field.
            elseif length(current_option) > 2 && ...
                strcmp(current_option(1:2), 'no') && ...
                isfield(opt, current_option(3:end))
            
                val = getfield(opt, current_option(3:end));
              
                % We only update the corresponding field if its attribute
                % is a logical value. For example, 'nobase' implies opt.base
                % = false
                if islogical(val)
                    % a logical variable can only be set by an option
                    opt = setfield(opt, current_option(3:end), false);
                    assigned = true;
                end
            end           
        end
        
        % non matching options are collected arglist returns the unrecognized 
        % options.
        if ~assigned
            % This is a non-exhaustive list of invalid command types.
            if ~ischar(options{argc})                
                if isnumeric(options{argc})
                    numeric_string = num2str(options{argc});
                    arglist = [arglist, ' ''[', numeric_string,']'','];
                elseif iscell(options{argc})
                    arglist = [arglist, ' ''', '<INVALID CELL>',''','];
                end
            else
                arglist = [arglist, ' ''', options(argc),''','];
            end            
        end
        argc = argc + 1;
    end % while

    % if enumerator value not assigned, set the default value
    for field=fieldnames(default)'
        if iscell(getfield(default, field{1})) && iscell(getfield(opt, field{1}))
            val = getfield(opt, field{1});
            if isempty(val{1})
                opt = setfield(opt, field{1}, val{1});
            elseif val{1}(1) == '#'
                opt = setfield(opt, field{1}, 1);
            else
                opt = setfield(opt, field{1}, val{1});
            end
        end
    end 

    others = arglist;    
end

function set_effector(robot,effector)
robot.effector = effector;
end
