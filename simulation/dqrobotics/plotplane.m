p1 = [1,0,0];
p2 = [0,1,0];
p3 = [0,0,1];
plot_line(p1, p2, p3);

function [normal, d] = plot_line(p1, p2, p3)
% This function plots a line from three points. 
% I/P arguments: 
%   p1, p2, p3 eg, p1 = [x y z]
% 
%
% O/P is: 
% normal: it contains a,b,c coeff , normal = [a b c]
% d : coeff
normal = cross(p1 - p2, p1 - p3);
d = p1(1)*normal(1) + p1(2)*normal(2) + p1(3)*normal(3);
d = -d;
x = -0.25:0.05:0.25; y = -0.25:0.05:0.25;
[X,Y] = meshgrid(x,y);
Z = (-d - (normal(1)*X) - (normal(2)*Y))/normal(3);
mesh(X,Y,Z)
end