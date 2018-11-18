clear all
close all
% Make Jacobian and Diff Jacobian for Robot and IMU

syms q1 q2 q3 real
q = [q1, q2, q3];
p = sym(pi()); 

imu.rcm2pos = [0 , 0.492, 0];

%% Kinematics
imu.a      =    [        0,         0,                   0];                    
imu.alpha  =    [      p/2,      -p/2,                 p/2];                   
imu.d      =    [        0,         0, q3 - imu.rcm2pos(2)];                   
imu.theta  =    [ q1 + p/2,  q2 - p/2,                   0];

l_rcc = 0.4318;
l_tool = 0.4162;
l_pitch2yaw = 0.0091;
l_yaw2ctrlpt = 0.0102;

rbt.a      =    [        0,         0,          0,      0,    0, l_pitch2yaw,            0];                    
rbt.alpha  =    [      p/2,      -p/2,        p/2,      0, -p/2,        -p/2,         -p/2];                   
rbt.d      =    [        0,         0, q3 - l_rcc, l_tool,    0,           0, l_yaw2ctrlpt];            
rbt.theta  =    [ q1 + p/2,  q2 - p/2,          0,      0, -p/2,        -p/2,            0];


imu = kinematics(imu); 
rbt = kinematics(rbt);

%% Make Jacobian
imu = jacobian(imu);
rbt = jacobian(rbt);

%% Put into ccode

stringname = 'imu_Ja.c';
ccode(imu.Ja,'File',stringname,'Comments','V1.2');

stringname = 'imu_Jd.c';
ccode(imu.Jd,'File',stringname,'Comments','V1.2');

stringname = 'rbt_Ja.c';
ccode(rbt.Ja,'File',stringname,'Comments','V1.2');

stringname = 'rbt_Jd.c';
ccode(rbt.Jd,'File',stringname,'Comments','V1.2');

