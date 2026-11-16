%% DC Motor Speed - System Analysis
% Analyze the motor BEFORE designing a controller.
% Input: armature voltage V(t) [V]
% Output: angular speed omega(t) [rad/s]
% Based on the CTMS DC Motor Speed: System Analysis tutorial.
% https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemAnalysis
% Requires Control System Toolbox.
%
% MATLAB tip: Add ; to hide a value in the Command Window.
% Remove ; when you want MATLAB to show the result.

%% 1. DC motor transfer function
J  = 0.01;
b  = 0.1;
Kt = 0.01;
Ke = 0.01;
R  = 1;
L  = 0.5;

s = tf('s');
P_motor = Kt/((J*s+b)*(L*s+R)+Kt*Ke)

%% 2. Open-loop step response (1 V)
% This is a voltage step, NOT a closed-loop speed reference.
linearSystemAnalyzer('step', P_motor, 0:0.1:5);

%% 3. Poles and first-order approximation
% In the analyzer: Right-click > Plot Types > Pole/Zero.
motor_poles = pole(P_motor)
motor_zeros = zero(P_motor)

% Keep the dominant pole (approximately -2) and the DC gain (~0.1).
rP_motor = 0.1/(0.5*s+1)

% Compare both step responses in the same analyzer window.
linearSystemAnalyzer('step', P_motor, rP_motor, 0:0.1:5);

%% 4. Sinusoidal input (0.2 Hz, amplitude 1 V, 0 to 5 sec)
% Convert 0.2 Hz to rad/s: 2*pi*0.2 = 1.2566 rad/s.
t = 0:0.1:5;
f = 0.2;                  % Frequency in Hz
u = sin(2*pi*f*t);        % Input voltage [V]
linearSystemAnalyzer('lsim', P_motor, rP_motor, u, t);

%% 5. Frequency-response view (Bode)
% Use the analyzer to inspect magnitude and phase vs frequency.
% Phase margin is not defined here: P_motor has no 0-dB crossover.
linearSystemAnalyzer('bode', P_motor);

% Optional: Right-click inside an analyzer plot to view Impulse,
% Pole/Zero, Nyquist, and other available plot types.
