%% DC Motor Speed - System Modeling
% Physics-based transfer function and state-space representations.
% Input:  V(t)     - armature voltage [V]
% Output: omega(t) - shaft angular speed [rad/s]
% State:  x = [omega; i] where i is armature current [A]
%
% Reference: CTMS, DC Motor Speed: System Modeling
% https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
% Requires MATLAB Control System Toolbox.

clc;
clear;

%% 1. Physical motor parameters
J  = 0.01;  % Rotor moment of inertia [kg*m^2]
b  = 0.1;   % Viscous friction coefficient [N*m*s/rad]
Kt = 0.01;  % Motor torque constant [N*m/A]
Ke = 0.01;  % Back-EMF constant [V/(rad/s)]
R  = 1;     % Armature resistance [ohm]
L  = 0.5;   % Armature inductance [H]

% In this CTMS example, Kt and Ke have equal numerical values in SI units.
% Keep them separate in code to show their different physical roles.

%% 2. Transfer-function representation
% Mechanical: J*d(omega)/dt + b*omega = Kt*i
% Electrical: L*di/dt + R*i = V - Ke*omega
% With zero initial conditions:
% P(s) = Omega(s)/V(s) = Kt/((J*s+b)*(L*s+R)+Kt*Ke)

s = tf('s');
P_motor = Kt / ((J*s + b)*(L*s + R) + Kt*Ke);

disp('=== Transfer Function: P_motor(s) = Omega(s)/V(s) ===');
disp(P_motor);

% Expanded denominator coefficients, in descending powers of s:
% [J*L, J*R + b*L, b*R + Kt*Ke]
denominator_coeffs = [J*L, J*R + b*L, b*R + Kt*Ke];
fprintf('Expanded denominator: %.6g*s^2 + %.6g*s + %.6g\n', ...
    denominator_coeffs(1), denominator_coeffs(2), denominator_coeffs(3));

%% 3. State-space representation using PHYSICAL states
% Define x1 = omega and x2 = i.
%
% d(omega)/dt = -(b/J)*omega + (Kt/J)*i
% di/dt       = -(Ke/L)*omega - (R/L)*i + (1/L)*V
%
% x_dot = A*x + B*V;  y = C*x + D*V;  y = omega.

A = [-b/J,    Kt/J;
     -Ke/L,  -R/L];
B = [0;
     1/L];
C = [1, 0];
D = 0;

motor_ss = ss(A, B, C, D);

disp('=== State Space: x = [omega; i], y = omega ===');
disp(motor_ss);

%% 4. Convert state-space model to transfer function
% The two representations should describe the SAME input-output behavior.
P_from_ss = tf(motor_ss);

disp('=== Transfer Function Obtained from State Space ===');
disp(P_from_ss);

%% 5. Verify transfer function and state-space equivalence
% Compare the frequency-domain input-output responses at sample frequencies.
% This checks numerical equivalence without assuming identical state bases.
frequency_rad_s = [0, 0.1, 1, 2, 10, 100];
H_tf = freqresp(P_motor, frequency_rad_s);
H_ss = freqresp(motor_ss, frequency_rad_s);
max_absolute_error = max(abs(H_tf(:) - H_ss(:)));

tolerance = 1e-9;
fprintf('Maximum absolute TF vs SS response error: %.3e\n', max_absolute_error);
assert(max_absolute_error < tolerance, ...
    'Transfer-function and state-space models are not equivalent.');
disp('PASS: Transfer-function and physical state-space models match.');

%% 6. Optional reverse conversion: TF to state space
% MATLAB chooses a realization automatically; its A/B/C matrices need
% NOT match the physical-state realization with x = [omega; i].
ss_from_tf = ss(P_motor);

disp('=== An Automatic State-Space Realization from P_motor ===');
disp(ss_from_tf);
disp('Note: Equivalent input-output models may use different state coordinates.');

% No plots are generated here. Time/frequency analysis belongs to
% System/Analysis, while controller design belongs to Control/*.
