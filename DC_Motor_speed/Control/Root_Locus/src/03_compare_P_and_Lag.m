%% 1. DC Motor
clc;
clear;
close all;

s = tf('s');
P_motor = 2/(s^2 + 12*s + 20.02);

%% 2. Proportional baseline
K_P = 9.99;
T_P = feedback(K_P*P_motor,1);

%% 3. Analytically designed lag compensator
zeta = 0.9;
sigma = 4.5;
e_target = 0.005;
wn = sigma/zeta;
alpha = 12-2*sigma;
p = e_target*wn^2*alpha/(20.02-e_target*wn^2);
a = alpha+p;
K = (wn^2+2*sigma*a-20.02-12*p)/2;
z = (a*wn^2-20.02*p)/(2*K);
C_lag = K*(s+z)/(s+p);
T_lag = feedback(C_lag*P_motor,1);

%% 4. Compare step responses
linearSystemAnalyzer('step',T_P,T_lag,0:0.01:5);

%% 5. Compare numerical results
P_info = stepinfo(T_P)
Lag_info = stepinfo(T_lag)
P_error_percent = (1-dcgain(T_P))*100
Lag_error_percent = (1-dcgain(T_lag))*100
