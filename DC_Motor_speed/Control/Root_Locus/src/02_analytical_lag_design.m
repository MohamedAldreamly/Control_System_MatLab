%% 1. DC Motor
clc;
clear;
close all;

s = tf('s');
P_motor = 2/(s^2 + 12*s + 20.02);

%% 2. Targets (design choices WITHIN the specified limits)
zeta = 0.90;
sigma = 4.50;              % zeta * wn (rad/s)
e_target = 0.005;          % 0.5 percent step tracking error
wn = sigma/zeta;
wd = wn*sqrt(1-zeta^2);

%% 3. Exact third-order coefficient matching
% Desired polynomial: (s^2 + 2*sigma*s + wn^2)*(s+a)
% Actual: (s+p)*(s^2+12*s+20.02)+2*K*(s+z)
% Coefficient s^2: a = 12+p-2*sigma
% Step error: e_target = 20.02*p/(a*wn^2)

alpha = 12-2*sigma;
p = (e_target*wn^2*alpha)/(20.02-e_target*wn^2);
a = alpha+p;
K = (wn^2 + 2*sigma*a - 20.02 - 12*p)/2;
z = (a*wn^2 - 20.02*p)/(2*K);

%% 4. Lag compensator and feedback system
C_lag = K*(s+z)/(s+p)
T_lag = feedback(C_lag*P_motor,1)

%% 5. Theoretical-model verification in MATLAB
Lag_poles = pole(T_lag)
Lag_info = stepinfo(T_lag)
Lag_error_percent = (1-dcgain(T_lag))*100

%% 6. Interactive design display (no auto-tuning)
controlSystemDesigner('rlocus',P_motor,C_lag);
