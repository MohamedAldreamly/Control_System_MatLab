clc;
clear;
close all;

%% Stage 1: P controller designed from a target phase margin
% Plant: P(s) = 2/(s^2 + 12s + 20.02)
% Open loop: L(s) = K_P*P(s)
% Unity-feedback closed loop: T(s) = L(s)/(1+L(s))
s = tf('s');
P_motor = 2/(s^2 + 12*s + 20.02);

%% Design requirement: desired phase margin (degrees)
PM_target = 60;

%% Calculate gain crossover frequency w_gc
% At gain crossover: |L(j*w_gc)| = 1.
% With positive K_P: PM_target = 180 + angle(P(j*w_gc)).
% P(j*w) = 2/[(20.02-w^2) + j*12*w].
% Desired denominator phase = 180 - PM_target (degrees).
% Therefore: w^2 - 20.02 = 12*w*cot(PM_target).
% Positive root of w^2 - 12*cot(PM_target)*w - 20.02 = 0:
b = 12*cotd(PM_target);
w_gc = (b + sqrt(b^2 + 4*20.02))/2

%% Calculate proportional gain K_P
% |K_P*P(j*w_gc)| = 1 => K_P = 1/|P(j*w_gc)|.
% |P(j*w)| = 2/sqrt[(20.02-w^2)^2 + (12*w)^2].
K_P = sqrt((20.02-w_gc^2)^2 + (12*w_gc)^2)/2

%% Open-loop margins and closed-loop step response
L_P = K_P*P_motor;
T_P = feedback(L_P,1);

% stepinfo gives time-domain performance of the CLOSED loop.
P_info = stepinfo(T_P)

% Unit reference step: final output = dcgain(T_P).
% e_ss(percent) = [(1 - final_output)/1]*100.
P_error_percent = (1-dcgain(T_P))*100

% margin uses OPEN loop. Wcp = gain-crossover freq (rad/s).
% Wcg = phase-crossover freq (rad/s), associated with gain margin.
[Gm_P,Pm_P,Wcg_P,Wcp_P] = margin(L_P)

figure('Name','P Open Loop Bode');
margin(L_P);
grid on;

figure('Name','P Closed Loop Step');
step(T_P,5);
grid on;
