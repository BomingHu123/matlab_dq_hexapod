% resultqd_1 = corin.fkm(q);
resultq_1 =corin.fkm(q);
% Res_Xd = [];
Res_X_Current = [];
[r,c]= size(Q);
for i = 1:6
%     res_xd = translation(resultqd_1(i));
%     Res_Xd = [Res_Xd,res_xd];
    res_x_current = translation(resultq_1(i));
    Res_X_Current = [Res_X_Current,res_x_current];
end

Desired_Feet_to_Feet = zeros(6,c);
for i = 1:6
    vec_desired_feet_to_feet = vec3(translation(xd(i)));
    desired_feet1_to_feet2 = norm(vec_desired_feet_to_feet);
    Desired_Feet_to_Feet(i,1) = desired_feet1_to_feet2;
end


Feet1_to_Feet2 = zeros(6,c);
X_Current = [];



for i = 1:c
    x_current = corin.fkm(Q(:,i));
    X_Current = [X_Current; x_current];
    for j =1:6
        vec_feet1_to_feet2 = vec3(translation(X_Current(i,j)));
        feet1_to_feet2 = norm(vec_feet1_to_feet2);
        
        Feet1_to_Feet2(j,i) = feet1_to_feet2;
        Desired_Feet_to_Feet(j,i) = Desired_Feet_to_Feet(j,1);
    end
end

for i = 1:6
    
    figure(i);
    hold on;
    plot(Time,Feet1_to_Feet2(i,:));
    plot(Time,Desired_Feet_to_Feet(i,:));
    xlabel('time');
    ylabel('Euclidean norm')
    h = legend('x_{current}','x_{desired}');
end

    figure(7);
    hold on;
    plot(Time(1:269),C);
    xlabel('time');
    ylabel('Norm')
    h = legend('Error');