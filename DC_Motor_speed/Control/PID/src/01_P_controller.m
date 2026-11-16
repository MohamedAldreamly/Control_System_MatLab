% Stage 1: Analytical proportional controller (P)
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

%% Calculated P gain (zeta = 0.8)
zeta = 0.8;
wn = 12/(2*zeta);
Kp = (wn^2 - 20.02)/2

%% Closed-loop model
C_P = pid(Kp);
T_P = feedback(C_P*P_motor,1)

%% Step response (1 rad/s speed reference)
linearSystemAnalyzer('step',T_P,0:0.01:5);
