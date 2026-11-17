clc;
clear;
close all;

%% Stage 2: lag-only design from magnitude, phase and step-error targets
s = tf('s');
P_motor = 2/(s^2 + 12*s + 20.02);

% Targets are engineering choices to be checked against time specifications.
w_gc = 5;             % desired gain crossover frequency, rad/s
lag_phase = 15;       % desired lag phase is NEGATIVE 15 degrees
error_target = 0.005; % unit-step error target = 0.5 percent (as a fraction)

%% 1) Error condition -> required controller DC gain Q
% e_ss = 1/[1+L(0)], L(0) = P(0)*K*z/p.
% Q = K*z/p = [1/e_ss - 1]/P(0).
Q = (1/error_target-1)/dcgain(P_motor)

%% 2) Crossover magnitude condition -> required magnitude M
% |K*(j*w_gc+z)/(j*w_gc+p) * P(j*w_gc)| = 1.
% M = |K*(j*w_gc+z)/(j*w_gc+p)| = 1/|P(j*w_gc)|.
P_mag = abs(evalfr(P_motor,1j*w_gc));
M = 1/P_mag

%% 3) Phase condition -> known real and imaginary parts
% angle(K) = 0 for K>0.
% angle(j*w_gc+z) - angle(j*w_gc+p) = -lag_phase.
% Desired complex response: M*exp(-j*lag_phase) = R+j*I.
R = M*cosd(lag_phase);
I = -M*sind(lag_phase);

%% Solve analytically for lag pole, gain and zero
% From Q=K*z/p, rewrite K*(s+z)/(s+p) = K+(Q-K)*p/(s+p).
% Match real/imaginary parts at s=j*w_gc:
% p = w_gc*I/(R-Q), K = R+I*p/w_gc, z = Q*p/K.
p = w_gc*I/(R-Q)
K = R + I*p/w_gc
z = Q*p/K

% Lag must satisfy 0<p<z, with actual pole=-p, actual zero=-z.
assert(K>0 && p>0 && z>p,'Lag design constraints were not satisfied');

%% Construct the compensator and verify the complete closed loop
C_lag = K*(s+z)/(s+p);
L_lag = C_lag*P_motor;
T_lag = feedback(L_lag,1);

Lag_info = stepinfo(T_lag)
Lag_error_percent = (1-dcgain(T_lag))*100
[Gm_lag,Pm_lag,Wcg_lag,Wcp_lag] = margin(L_lag)

figure('Name','Lag Open Loop Bode');
margin(L_lag);
grid on;

figure('Name','Lag Closed Loop Step');
step(T_lag,5);
grid on;
