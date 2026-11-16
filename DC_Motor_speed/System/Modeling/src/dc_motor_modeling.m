%% DC Motor Speed - System Modeling
% Input:  V(t)     - armature voltage [V]
% Output: omega(t) - rotor angular speed [rad/s]
% Physical states: x1 = omega, x2 = i (armature current)
%
% Reference: CTMS - DC Motor Speed: System Modeling
% https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
% Requires the Control System Toolbox.
%
% MATLAB tip: A semicolon (;) HIDES a value in the Command Window.
% Remove the semicolon from a variable if you want to display it.

%% 1. Physical parameters
J  = 0.01;   % Rotor inertia [kg*m^2]
b  = 0.1;    % Viscous friction [N*m*s/rad]
Kt = 0.01;   % Torque constant [N*m/A]
Ke = 0.01;   % Back-EMF constant [V/(rad/s)]
R  = 1;      % Armature resistance [ohm]
L  = 0.5;    % Armature inductance [H]

%% 2. Transfer function (voltage -> speed)
% Mechanical: J*d(omega)/dt + b*omega = Kt*i
% Electrical: L*di/dt + R*i = V - Ke*omega
% P(s) = Omega(s)/V(s) = Kt / ((J*s+b)*(L*s+R)+Kt*Ke)

s = tf('s');
P_motor = Kt/((J*s+b)*(L*s+R)+Kt*Ke)

%% 3. State-space model using physical states
% x1 = omega, x2 = i
% d(omega)/dt = -(b/J)*omega + (Kt/J)*i
% di/dt       = -(Ke/L)*omega - (R/L)*i + (1/L)*V
% y = omega

A = [-b/J,    Kt/J;
     -Ke/L,  -R/L];
B = [0;
     1/L];
C = [1, 0];
D = 0;

motor_ss = ss(A,B,C,D)

%% 4. Convert state space to transfer function
% This should give the same voltage-to-speed transfer function.
% MATLAB may normalize the numerator and denominator coefficients.

P_from_ss = tf(motor_ss)

%% 5. Convert transfer function to state space
% MATLAB selects its own state coordinates in this conversion.
% Its A, B, C matrices may differ from the physical state-space model.

ss_from_tf = ss(P_motor)

% End of modeling. Step response and frequency plots are in System/Analysis.
