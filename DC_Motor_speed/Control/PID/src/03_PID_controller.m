% Stage 3: Analytical PID controller (pole placement)
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

%% Design choices: dominant pair -4 +/- 3j; extra pole -25
zeta = 0.8;
wn = 5;
a = 25;

%% Coefficient matching: (s^2 + 2*zeta*wn*s + wn^2)*(s+a)
Kd = (2*zeta*wn + a - 12)/2
Kp = (wn^2 + 2*a*zeta*wn - 20.02)/2
Ki = a*wn^2/2

%% Closed-loop model
C_PID = pid(Kp,Ki,Kd);
T_PID = feedback(C_PID*P_motor,1)

%% Root-locus and step response in Control System Designer
controlSystemDesigner('rlocus',P_motor,C_PID);

%% Optional: inspect the step response in Linear System Analyzer
% linearSystemAnalyzer('step',T_PID,0:0.01:5);
