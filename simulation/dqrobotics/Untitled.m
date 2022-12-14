% syms l1
% 
% a = [0,0,1];
% b = [0,-3*l1,0];
% -cross(a,b)

clc,clear,close all;

h=-25:0;

[x,y,z]=cylinder(h);

x=x+rand(26,21);

y=y+rand(26,21);

surfl(x,y,z*5+1)

colormap lines

hold on?

cylinder(5);

view(40,10);

axis off;

[x,y]=deal(rand(50,1)*40-20);

z=rand(50,1)*5;

scatter3(x,y,z,z*20,'rp','filled');

scatter3(1,0,6,1200,'rp','filled','MarkerFaceColor','y', 'MarkerEdgeColor','k');

shading flat

text(-9,-9,7,'\it{Merry Christmas}','fontsize',15);