function imu = kinematics(imu)
syms q1 q2 q3 real
p = sym(pi()); 
q = [q1, q2, q3];

for i=1:length(imu.a)
    imu.Ti_i(1:4,1:4,i) = modified_dh(imu.theta(i),imu.a(i),imu.d(i),imu.alpha(i));
end

for i=1:length(imu.a)
    if i==1
        imu.T(1:4,1:4,i) = imu.Ti_i(:,:,i);
    else
        imu.T(1:4,1:4,i)=imu.T(1:4,1:4,i-1)*imu.Ti_i(1:4,1:4,i);
    end
end

%% Plot Joint Angles Test
close all
q_n = [0.0, 0.5, 0.0];  %Put your numeric values here

figure()
imu_shape = size(imu.T);
dof = imu_shape(3);
for i = 1:dof
        T_num(:,:,i)=subs(imu.T(:,:,i), q, q_n);
        
        scatter3(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
        marker_id = sprintf('%d',i);
        line(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i));
        text(T_num(1,4,i),T_num(2,4,i),T_num(3,4,i),marker_id);
        hold on
end

imu.T_num=T_num;
title('Plot transform Frames');
xlabel('x');
ylabel('y');
zlabel('z');

hold off

end