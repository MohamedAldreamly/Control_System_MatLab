% Open the pre-calculated PID in MATLAB Control System Designer
clc;
clear;
close all;

%% DC motor model
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;
s = tf('s');
P_motor = K/((J*s+b)*(L*s+R)+K^2);

%% Controller derived in the README
Kp = 102.49;
Ki = 312.5;
Kd = 10.5;
C_PID = pid(Kp,Ki,Kd);

%% Root locus editor with compensator C_PID
controlSystemDesigner('rlocus',P_motor,C_PID);
