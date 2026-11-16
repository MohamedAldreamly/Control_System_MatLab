%% 1. DC Motor (open-loop plant)
clc;
clear;
close all;

s = tf('s');
P_motor = 2/(s^2 + 12*s + 20.02);

%% 2. Proportional baseline from desired poles near -6 +/- 2j
K_P = (19.98)/2;
C_P = K_P;
T_P = feedback(C_P*P_motor,1);

%% 3. Performance
P_poles = pole(T_P)
P_info = stepinfo(T_P)
P_error_percent = (1-dcgain(T_P))*100

%% 4. Control System Designer: plant and calculated proportional gain
controlSystemDesigner('rlocus',P_motor,C_P);
