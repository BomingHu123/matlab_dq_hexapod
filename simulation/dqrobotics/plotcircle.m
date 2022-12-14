% clear,clc
% close all
% % %% 定义初始圆的参数
% % % 定义3维圆圈的圆心坐标
% % O = [0, 0, 0];
% % % 定义半径的大小
% % r = 2;
% % % 定义法向向量
% % n = [1, 1, 2];
% % %% 开始绘制3维圆圈
% % theta = linspace(0, 2*pi, 100);
% % x = O(1)+r*n(2)/sqrt(n(1)^2+n(2)^2).*cos(theta)+...
% % 	n(1)*n(2)/(sqrt(n(1)^2+n(2)^2)*sqrt(n(1)^2+n(2)^2+n(3)^2)).*sin(theta);
% % y = O(2)-r*n(1)/sqrt(n(1)^2+n(2)^2).*cos(theta)+...
% % 	n(2)*n(3)/(sqrt(n(1)^2+n(2)^2)*sqrt(n(1)^2+n(2)^2+n(3)^2)).*sin(theta);
% % z = O(3)-r*sqrt(n(1)^2+n(2)^2)/sqrt(n(1)^2+n(2)^2+n(3)^2).*sin(theta);
% % plot3(x,y,z,'r-', 'LineWidth', 1.3)
% % hold on
% 
% t = linspace(0, 2*pi);
% r = 1;
% x = r*cos(t);
% y = r*sin(t);
% figure(1)
% patch(x, y, 'g')
% axis equal


pos = rand(3,1);
rad = 1;R = eye(3);
drawCircle(rad,pos,R(:,1),'r')
hold on
drawCircle(rad,pos,R(:,2),'g')
drawCircle(rad,pos,R(:,3),'b')
axis equal   

function drawCircle(rad,pos,n,color)
    %https://demonstrations.wolfram.com/ParametricEquationOfACircleIn3D/
    %draws a 3D circle at position pos with radius rad, normal to the
    %circle n, and color color.
    phi = atan2(n(2),n(1)); %azimuth angle, in [-pi, pi]
    theta = atan2(sqrt(n(1)^2 + n(2)^2) ,n(3));% zenith angle, in [0,pi]    
    t = 0:pi/32:2*pi;
    x = pos(1)- rad*( cos(t)*sin(phi) + sin(t)*cos(theta)*cos(phi) );
    y = pos(2)+ rad*( cos(t)*cos(phi) - sin(t)*cos(theta)*sin(phi) );
    z = pos(3)+ rad*sin(t)*sin(theta);
    plot3(x,y,z,color)
end
    
% then call the function as 
