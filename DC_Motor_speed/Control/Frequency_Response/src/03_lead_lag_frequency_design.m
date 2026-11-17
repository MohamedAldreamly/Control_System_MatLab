clc;
clear;
close all;

%% Stage 3: analytical Lead-Lag frequency-response design
% Total controller: C(s) = K*(s+z_lag)/(s+p_lag) * (1+tau*s)/(1+alpha*tau*s)
s = tf('s');
P_motor = 2/(s^2 + 12*s + 20.02);

%% Design targets (engineering choices, not uniquely implied by step limits)
w_gc = 10;             % target gain crossover, rad/s
PM_target = 65;        % target phase margin, degrees
lag_phase_budget = 5; % lag contributes NEGATIVE 5 deg at crossover
e_target = 0.005;      % 0.5% unit-step steady-state error

%% 1) Evaluate the original plant at the selected crossover
% P(j*w_gc): magnitude = abs(P_w), phase = angle(P_w) in radians.
P_w = evalfr(P_motor,1j*w_gc);
P_mag = abs(P_w);
P_phase = angle(P_w)*180/pi;

%% 2) Calculate needed LEAD phase from the total phase budget
% PM_target = 180 + plant_phase + lead_phase - lag_phase_budget.
% lead_phase = PM_target - (180 + plant_phase) + lag_phase_budget.
lead_phase = PM_target-(180+P_phase)+lag_phase_budget

%% 3) Calculate first-order lead parameters alpha and tau
% C_lead(s) = (1+tau*s)/(1+alpha*tau*s), 0<alpha<1.
% sin(lead_phase) = (1-alpha)/(1+alpha).
% alpha = (1-sin(lead_phase))/(1+sin(lead_phase)).
alpha = (1-sind(lead_phase))/(1+sind(lead_phase))

% Peak lead phase at w_gc: w_gc = 1/[tau*sqrt(alpha)].
% Therefore tau = 1/[w_gc*sqrt(alpha)].
tau = 1/(w_gc*sqrt(alpha))
C_lead = (1+tau*s)/(1+alpha*tau*s);

%% 4) DC error requirement -> Q (gain of Lag+K at zero frequency)
% e_ss = 1/(1+L(0)). For normalized lead, C_lead(0)=1.
% Therefore Q = K*z_lag/p_lag = [1/e_target-1]/P(0).
Q = (1/e_target-1)/dcgain(P_motor)

%% 5) Magnitude condition at crossover -> magnitude M of Lag+K
% |L(j*w_gc)| = 1.
% M = |K*(j*w_gc+z_lag)/(j*w_gc+p_lag)|
%   = 1/[|P(j*w_gc)|*|C_lead(j*w_gc)|].
lead_mag = abs(evalfr(C_lead,1j*w_gc));
M = 1/(P_mag*lead_mag)

%% 6) Lag angle condition -> required complex response R+j*I
% angle(K*Lag(j*w_gc)) = -lag_phase_budget degrees.
% R = M*cos(lag_phase_budget); I = -M*sin(lag_phase_budget).
R = M*cosd(lag_phase_budget);
I = -M*sind(lag_phase_budget);

%% 7) Solve for lag pole, overall gain and lag zero
% Using Q=K*z_lag/p_lag:
% K*(s+z_lag)/(s+p_lag) = K+(Q-K)*p_lag/(s+p_lag).
% Equating its real/imaginary parts at s=j*w_gc gives:
% p_lag = w_gc*I/(R-Q)
% K = R + I*p_lag/w_gc
% z_lag = Q*p_lag/K
p_lag = w_gc*I/(R-Q)
K = R + I*p_lag/w_gc
z_lag = Q*p_lag/K

assert(0<alpha && alpha<1,'Lead needs 0 < alpha < 1');
assert(K>0 && p_lag>0 && z_lag>p_lag,'Lag needs K>0 and 0<p<z');

%% 8) Assemble total controller; analyze OPEN and CLOSED loops
C_lag = (s+z_lag)/(s+p_lag);
C_total = K*C_lag*C_lead
L_total = C_total*P_motor;      % for Bode and stability margins
T_total = feedback(L_total,1); % for step response and tracking error

LeadLag_info = stepinfo(T_total)
% Unit reference step: final output = dcgain(T_total).
LeadLag_error_percent = (1-dcgain(T_total))*100
LeadLag_poles = pole(T_total)
% Wcp is gain crossover (frequency of phase-margin measurement).
[Gm_total,Pm_total,Wcg_total,Wcp_total] = margin(L_total)

figure('Name','Lead Lag Open Loop Bode');
margin(L_total);
grid on;

figure('Name','Lead Lag Closed Loop Step');
step(T_total,5);
grid on;

% Optional manual inspection:
% controlSystemDesigner('bode',P_motor,C_total);
