% Stage 2: Analytical PI controller (triple pole at -4)
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

%% Coefficient matching: (s+4)^3 = s^3+12s^2+48s+64
Kp = (48 - 20.02)/2
Ki = 64/2

%% Closed-loop model
C_PI = pid(Kp,Ki);
T_PI = feedback(C_PI*P_motor,1)

%% Step response (1 rad/s speed reference)
linearSystemAnalyzer('step',T_PI,0:0.01:5);
