%% DC Motor - Steady-State Error Constants (Textbook Example)
% IMPORTANT: The classical step/ramp/parabolic tracking errors below
% assume a hypothetical UNITY NEGATIVE FEEDBACK loop with C(s) = 1.
% They are NOT the same as the open-loop response to a 1-V step.
% No PID controller is designed in this script.

%% 1. Original motor model
J  = 0.01;
b  = 0.1;
Kt = 0.01;
Ke = 0.01;
R  = 1;
L  = 0.5;

s = tf('s');
P_motor = Kt/((J*s+b)*(L*s+R)+Kt*Ke);

%% 2. Error constants for a unity-feedback system
% This is a type-0 loop (no integrator in P_motor).
K_position = dcgain(P_motor)
K_velocity = dcgain(s*P_motor)
K_acceleration = dcgain(s^2*P_motor)

%% 3. Steady-state tracking errors
% Step: r(t)=1; Ramp: r(t)=t; Parabolic: r(t)=t^2/2.
ess_step = 1/(1+K_position)
ess_ramp = Inf           % K_velocity = 0
ess_parabolic = Inf      % K_acceleration = 0
