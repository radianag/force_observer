function imu = jacobian(imu)
% Make Jacobian

syms q1 q2 q3 real
q = [q1, q2, q3];
p = sym(pi()); 
syms q1 q2 q3 real
syms  q1t(t) q2t(t) q3t(t) 
syms qd1 qd2 qd3 

imu.Ja=[diff(imu.T(1:3,4,end), q(1)), diff(imu.T(1:3,4,end), q(2)), diff(imu.T(1:3,4,end), q(3))];

qd = [qd1 qd2 qd3 ];
qt = [q1t(t) q2t(t) q3t(t)];

imu.Ja_t = subs(imu.Ja, q, qt);
imu.Jd_t = diff(imu.Ja_t,t);

imu.Jd = subs(imu.Jd_t, [diff(q1t,t),diff(q2t,t),diff(q3t,t)],qd);
imu.Jd = subs(imu.Jd, q1t, q(1));
imu.Jd = subs(imu.Jd, q2t, q(2));
imu.Jd = subs(imu.Jd, q3t, q(3));

%% Plot Jacobian
p = pi();
q_n = [0.5, 0, 0.1];

J3_num = subs(imu.Ja,q1,q_n(1));
J3_num = subs(J3_num,q2,q_n(2));
J3_num = subs(J3_num,q3,q_n(3));

J3_num = double(J3_num);

figure()
x0 = [0, 0 ,0];
 xv=J3_num*transpose([10 0 0]);
 
 xvx=xv
 xd = x0+xv(1:3);
 plot3([xd(1), 0],[xd(2), 0],[xd(3),0 ],'r');
 hold on
 xv=J3_num*transpose([0 10 0]);
 
 xvy=xv
 xd = x0+xv(1:3);
 plot3([xd(1),0 ],[xd(2),0],[xd(3),0],'g');
 xv=J3_num*transpose([0 0 0.5]);
 xd = x0+xv(1:3);
 plot3([xd(1),0],[xd(2),0],[xd(3),0],'b');
 

 title('Jacobian');
 xlabel('x');
 ylabel('y');
 zlabel('z');
 s=1;
 xlim([-s/2 s])
 ylim([-s/2 s])
 zlim([-s/2 s])


end
