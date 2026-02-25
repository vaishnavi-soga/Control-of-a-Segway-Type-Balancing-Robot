%% =========================================================================
%  segway_model.m
%  Defines the linearized state-space model of the Segway robot.
%  Run this first, or call it from other scripts.
%
%  Physical Parameters → Inertia matrix → Inverted → A, B, C, D matrices
%
%  STATES:   x1 = θ    body tilt angle         [rad]
%            x2 = θ̇    body angular velocity   [rad/s]
%            x3 = φ̇    wheel angular velocity  [rad/s]
%  INPUT:    u  = motor torque                  [N·m]
%  OUTPUT:   y  = θ    (body angle only)
% =========================================================================

%% Physical Parameters (Table 2.4 in report)
r   = 0.085;   % Wheel radius              [m]
m_w = 0.45;    % Wheel mass                [kg]
I_w = 0.0012;  % Wheel rotational inertia  [kg·m²]
m_b = 3.20;    % Body mass                 [kg]
I_b = 0.045;   % Body inertia              [kg·m²]
h   = 0.24;    % Center-of-mass height     [m]
g   = 9.81;    % Gravity                   [m/s²]
b   = 0.11;    % Viscous friction          [N·m·s]

%% Inertia matrix constants (Section 2.5)
% A2 = [a  c; -c  d]  where:
a = I_b + m_b*h^2 + m_b*r*h;   % M_theta
c = m_b * h * r;                 % coupling term
d = I_w + (m_w + m_b)*r^2;      % M_w

det_A2 = a*d + c^2;

fprintf('Derived constants:\n');
fprintf('  a = M_theta = %.5f\n', a);
fprintf('  c = m_b*h*r = %.5f\n', c);
fprintf('  d = M_w     = %.8f\n', d);
fprintf('  det(A2)     = %.11f\n\n', det_A2);

%% State-Space Matrices
% Using exact numeric values from report derivation:
A = [0,                     1,                      0;
    -6.34883678730897,       0,       1.35942351946378;
    -93.1091413592878,       0,      -4.77547490017514];

B = [0;
    -12.3583956314889;
     43.4134081834104];

C = [1, 0, 0];
D = 0;

%% Create state-space system object
sys_ol = ss(A, B, C, D);

fprintf('State-Space Model Created:\n');
fprintf('A =\n'); disp(A);
fprintf('B =\n'); disp(B);
fprintf('C = [1 0 0],  D = 0\n\n');
fprintf('Open-loop eigenvalues: '); disp(eig(A).');
