% Compare the three analytically calculated controllers
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

%% Calculated gains
C_P = pid(18.115);
C_PI = pid(13.99,32);
C_PID = pid(102.49,312.5,10.5);

%% Closed-loop systems
T_P = feedback(C_P*P_motor,1);
T_PI = feedback(C_PI*P_motor,1);
T_PID = feedback(C_PID*P_motor,1);

%% Compare step responses
linearSystemAnalyzer('step',T_P,T_PI,T_PID,0:0.01:5);

%% View numerical metrics in MATLAB Command Window
P_Info = stepinfo(T_P)
PI_Info = stepinfo(T_PI)
PID_Info = stepinfo(T_PID)

P_Error_percent = (1-dcgain(T_P))*100
PI_Error_percent = (1-dcgain(T_PI))*100
PID_Error_percent = (1-dcgain(T_PID))*100

P_Poles = pole(T_P)
PI_Poles = pole(T_PI)
PID_Poles = pole(T_PID)
