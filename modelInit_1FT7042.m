clear all;
clc;

%% Inputs to the switches in the Simulink model
% theta_true = 1 when running P-STSMC controller (otherwise 0)
theta_true = 1;

% theta_step_true = 1 when running step input for theta_r (0 for sine
% input)
theta_step_true = 1;    % doesn't matter when theta_true = 0

% theta_dot_zero_true = 0;
% if theta_true == 1 && theta_step_true == 1
%     theta_dot_zero_true = 1;
% end

% omega_step_true = 1 when running step input for omega_r (0 for sine
% input)
omega_step_true = 0;

% Frequency for sine wave
xf = 1;

%% Controller parameters
% P-controller parameter
k_pos = 21.1755;

% STSMC (in nonlinear controller for omega_m)
k1 = 6.2113;
k2 = 9.9930;

%% Parameters for drive train
% This is the initialization script for the motor and axle parameters. Both
% motors are identical PMSM 1FT7042-5AF70-1DA0 HD.

% Motor electrical parameters
% r_s = 3.6644;           % Ohm -- Stator winding resistance (per phase)
% L_d = 21.4e-3;          % mH -- Rotating field inductance
% L_q = 1.2*L_d;          % mH -- Rotating torque inductance
% P = 6;                  % Non-dimensional -- Number of poles
% k_T = 1.43;             % Nm/A -- Torque constant
% k_E = 87*2*pi*60*0.001; % Vs/rad -- Voltage constant
% lambda_m = 0.3148;      % Vs/rad -- amplitude of the flux linkages
%                         %   established by the permanent magnet as viewed
%                         %   from the stator phase windings.
% Motor mechanical parameters
J_m = 2.81e-4 + 5.5e-4; % kgm^2 -- Moment of inertia
N = 1;                  % -- Gear ratio

% Saturation
u_max = 13;             % Nm -- Maximum torque

% Common simulation parameters
T_s = 0.000125; 		% Sampling time for control loops. For data acquisition, it is 0.125 ms
% T_s = 0.001;
T_log = T_s; % har bare valgt en video der er større end T_s
% we we use Ts instead of T_log it might give way too many samples

%% Values of friction and shaft parameters
% Taken from Table 4.3: Summary of calculated friction and shaft parameters
% (page 40, Dimitrios Papageorgiou phd thesis)

% Shaft constants
K_S = 32.94;    % N m rad^(-1)
D_S = 0.0548;   % N m s rad^(-1)

% Coulomb friction
% (assuming T_C is the average of T_C_m and T_C_l)
T_C = (0.0223 + 0.0232) / 2;    % N m

% Static friction
% (assuming T_S is the average of T_S_m and T_S_l)
T_S = (0.0441 + 0.0453) / 2;    % N m

% Friction constants
b_fr = 0.0016;  % N m s rad^(-1)

% Load inertia      (not sure...)
J_l = 1; % kgm^2 -- Moment of inertia
inv_J_l = J_l;

% Initial conditions vector (should be zero)
x_0 = [0,0];
x_l_0 = [0,0];

%% Simulink simulation - STSMC and P-STSMC hand-tuning
driveTrain_sim = sim("driveTrain_P_STSMC", 10);

h1 = figure(1);

if theta_true == 0
    if omega_step_true == 1
        plot(driveTrain_sim.omega_r_out, 'LineWidth', 1);
        hold on;
        plot(driveTrain_sim.omega_m_out, 'LineWidth', 1);
        hold on;
        yline(1.06, ':k');
        %hold on;
        %yline(0.94, ':k');
        hold on;
        yline(1.02, '--k');
        hold on;
        yline(0.98, '--k');
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('ang. velocity (rad/s)');
        legend('\omega_r', '\omega_m', 'Location', 'southeast');
        title('Simulink simulation of step response');
        saveas(h1, 'step response of STSMC hand-tuning.png');
    else
        plot(driveTrain_sim.omega_r_out, 'LineWidth', 1);
        hold on;
        plot(driveTrain_sim.omega_m_out, 'LineWidth', 1);
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('ang. velocity (rad/s)');
        legend('\omega_r', '\omega_m', 'Location', 'southeast');
        title('Simulink simulation of sine response');
        saveas(h1, 'sine response of STSMC hand-tuning.png');
    end
else
    if theta_step_true == 1
        plot(driveTrain_sim.theta_r_out, 'LineWidth', 1);
        hold on;
        plot(driveTrain_sim.theta_l_out, 'LineWidth', 1);
        hold on;
        yline(1.06, ':k');
        %hold on;
        %yline(0.94, ':k');
        hold on;
        yline(1.02, '--k');
        hold on;
        yline(0.98, '--k');
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('position (rad)');
        legend('\theta_r', '\theta_l', 'Location', 'southeast');
        title('Simulink simulation of step response');
        saveas(h1, 'step response of P-STSMC hand-tuning.png');
    else
        plot(driveTrain_sim.theta_r_out, 'LineWidth', 1);
        hold on;
        plot(driveTrain_sim.theta_l_out, 'LineWidth', 1);
        hold off;
        grid on;
        xlabel('time (s)');
        ylabel('position (rad)');
        legend('\theta_r', '\theta_l', 'Location', 'southeast');
        title('Simulink simulation of sine response');
        % saveas(h1, 'sine response of P-STSMC hand-tuning.png');
        saveas(h1, 'autotune_check_P-STSMC_1.png');
    end
end

%%
disp('Ran modelInit_1Ft7042.m file');
