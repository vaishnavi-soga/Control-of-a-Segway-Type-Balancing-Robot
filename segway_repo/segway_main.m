%% =========================================================================
%  MAE 506 — Segway-Type Balancing Robot
%  MAIN SCRIPT: Full analysis and control pipeline
%
%  Authors: Abhay Parwal, Vaishanavi Sogalad, Josia Vargheese Thomas
%  Arizona State University
%
%  Run this single file to reproduce ALL results from the report.
%  Each %% section can also be run independently in MATLAB.
%
%  Sections:
%    1. Physical Parameters
%    2. State-Space Model Construction
%    3. Controllability & Observability
%    4. Stability Analysis (Lyapunov + BIBO)
%    5. Phase Portrait (3D open-loop)
%    6. Open-Loop Step Response & Root Locus
%    7. Controller Design via Pole Placement
%    8. Closed-Loop Step Response
%    9. State Trajectories for Initial Tilt
% =========================================================================

clear; clc; close all;

fprintf('=============================================================\n');
fprintf('  MAE 506 — Segway-Type Balancing Robot\n');
fprintf('  State-Space Modeling and Control\n');
fprintf('=============================================================\n\n');

%% =========================================================================
%  SECTION 1: Physical Parameters
% =========================================================================
r   = 0.085;   % Wheel radius              [m]
m_w = 0.45;    % Wheel mass                [kg]
I_w = 0.0012;  % Wheel rotational inertia  [kg·m²]
m_b = 3.20;    % Body (pendulum) mass      [kg]
I_b = 0.045;   % Body rotational inertia   [kg·m²]
h   = 0.24;    % Center-of-mass height     [m]
g   = 9.81;    % Gravitational accel.      [m/s²]
b   = 0.11;    % Viscous friction          [N·m·s]

%% =========================================================================
%  SECTION 2: State-Space Model  (Section 2.5 in report)
%
%  States:  x1 = θ   (body tilt angle)
%           x2 = θ̇   (body angular velocity)
%           x3 = φ̇   (wheel angular velocity)
%  Input:   u = motor torque [N·m]
%  Output:  y = θ
%
%  Derived by inverting the inertia matrix A2 from Lagrangian mechanics.
%  Exact numeric values from report derivation:
% =========================================================================

A = [0,                     1,                      0;
    -6.34883678730897,       0,       1.35942351946378;
    -93.1091413592878,       0,      -4.77547490017514];

B = [0;
    -12.3583956314889;
     43.4134081834104];

C = [1, 0, 0];   % Output: measure body angle θ only
D = 0;

sys_ol = ss(A, B, C, D);

fprintf('State Matrix A =\n'); disp(A);
fprintf('Input Matrix B =\n'); disp(B);
fprintf('Output Matrix C = [1 0 0],  D = 0\n\n');

%% =========================================================================
%  SECTION 3: Controllability and Observability
% =========================================================================

% Controllability: rank([B AB A²B]) must equal n=3
Ctrl = [B, A*B, A^2*B];
rank_ctrl = rank(Ctrl);
fprintf('Controllability rank = %d / 3  -->  ', rank_ctrl);
if rank_ctrl == 3
    fprintf('CONTROLLABLE ✓\n');
else
    fprintf('NOT controllable ✗\n');
end

% Observability: rank([C; CA; CA²]) must equal n=3
Obs = [C; C*A; C*A^2];
rank_obs = rank(Obs);
fprintf('Observability rank  = %d / 3  -->  ', rank_obs);
if rank_obs == 3
    fprintf('OBSERVABLE ✓\n\n');
else
    fprintf('NOT observable ✗\n\n');
end

%% =========================================================================
%  SECTION 4: Stability Analysis
%
%  Lyapunov: all eig(A) must have Re < 0
%  BIBO:     all poles of transfer function in Left Half Plane (LHP)
%
%  Expected eigenvalues:
%    λ1 ≈ -7.04          (stable — decaying direction)
%    λ2,3 ≈ 1.13 ± 4.58j (UNSTABLE — growing oscillations = robot tips over)
% =========================================================================

ev = eig(A);
fprintf('Open-Loop Eigenvalues:\n');
for i = 1:3
    fprintf('  λ%d = %+.8f %+.8fj\n', i, real(ev(i)), imag(ev(i)));
end

fprintf('\nLyapunov Stability: ');
if all(real(ev) < 0)
    fprintf('Stable\n');
else
    fprintf('NOT stable ✗  (positive real parts → robot tips over without control)\n');
end

[~, den] = tfdata(sys_ol, 'v');
fprintf('BIBO Stability:     ');
if all(real(roots(den)) < 0)
    fprintf('Stable\n\n');
else
    fprintf('NOT stable ✗\n\n');
end

%% =========================================================================
%  SECTION 5: Phase Portrait — 3D Open-Loop
%
%  Shows trajectories starting from various initial conditions.
%  Outward spirals = system diverges from upright (unstable saddle-focus).
% =========================================================================

fprintf('Generating Phase Portrait...\n');
f_ol = @(t,x) A*x;
tspan = linspace(0, 3, 400);
theta_ic    = linspace(-0.18, 0.18, 6);
thetadot_ic = linspace(-0.8,  0.8,  6);

figure('Name','Phase Portrait','Position',[50 50 750 550]);
hold on; grid on;
xlabel('\theta (rad)'); ylabel('\dot{\theta} (rad/s)'); zlabel('\dot{\phi} (rad/s)');
title('3D Phase Portrait — Open-Loop (Outward Spiral = Unstable)','FontSize',13);
view(42,28); set(gca,'FontSize',11);
cmap = parula(length(tspan));

for th0 = theta_ic
    for dth0 = thetadot_ic
        [~, x] = ode45(f_ol, tspan, [th0; dth0; 0]);
        for k = 1:length(tspan)-1
            plot3(x(k:k+1,1), x(k:k+1,2), x(k:k+1,3), ...
                  'Color', cmap(k,:), 'LineWidth', 1.6);
        end
    end
end
plot3(0,0,0,'ko','MarkerFaceColor','k','MarkerSize',8);
colormap(parula);
colorbar('Ticks',[0 1],'TickLabels',{'t=0','t=end'});
hold off;

%% =========================================================================
%  SECTION 6: Open-Loop Step Response & Root Locus
% =========================================================================

figure('Name','Open-Loop Step Response');
step(sys_ol, 0:0.01:5);
title('Open-Loop Step Response — θ diverges (system unstable without control)');
xlabel('Time (s)'); ylabel('\theta (rad)'); grid on;

figure('Name','Root Locus — Open Loop');
rlocus(tf(sys_ol));
title('Root Locus of Open-Loop System KG(s)'); grid on;

%% =========================================================================
%  SECTION 7: Controller Design — Pole Placement  (Section 4.2)
%
%  Specifications:
%    - ~5% overshoot   → damping ratio ζ ≈ 0.69
%    - ~2s settling    → natural frequency ωn ≈ 2.90 rad/s
%
%  Desired closed-loop poles:
%    s1,2 = -2 ± 2j   (dominant pair for OS + settling specs)
%    s3   = -5         (fast real pole, doesn't dominate response)
%
%  Control law: u = -Kx
%  Result:      ẋ = (A - BK)x  → all poles in LHP → stable!
% =========================================================================

desired_poles = [-2+2j, -2-2j, -5];
K = place(A, B, desired_poles);

fprintf('--- Pole Placement Controller ---\n');
fprintf('Desired poles: -2±2j  (5%% OS, 2s settling),  -5  (fast)\n');
fprintf('Gain vector K = [%.4f  %.4f  %.4f]\n\n', K(1), K(2), K(3));

A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);

ev_cl = eig(A_cl);
fprintf('Closed-Loop Eigenvalues:\n');
for i = 1:3
    fprintf('  λ%d = %+.4f %+.4fj\n', i, real(ev_cl(i)), imag(ev_cl(i)));
end
fprintf('\nAll poles in LHP → Closed-Loop STABLE ✓\n\n');

%% =========================================================================
%  SECTION 8: Closed-Loop Step Response  (Section 4.3)
% =========================================================================

figure('Name','Closed-Loop Step Response');
step(sys_cl, 0:0.01:10);
title('Closed-Loop Step Response (Output \theta) — ~5% OS, ~2s Settling');
xlabel('Time (s)'); ylabel('\theta (rad)'); grid on;

figure('Name','Root Locus — Closed Loop');
rlocus(tf(sys_cl));
title('Root Locus of Closed-Loop System'); grid on;

%% =========================================================================
%  SECTION 9: State Trajectories — Initial 1° Tilt  (Section 4.4)
%
%  Simulates closed-loop response from x0 = [1°, 0, 0]
%  Shows how the controller recovers all states back to zero.
% =========================================================================

x0    = [pi/180; 0; 0];   % 1-degree tilt, zero velocities
t_sim = 0:0.005:4;

[~, t_out, x_out] = initial(sys_cl, x0, t_sim);

figure('Name','State Trajectories','Position',[50 50 1100 380]);

subplot(1,3,1);
plot(t_out, x_out(:,1)*180/pi, 'b-', 'LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('\theta (deg)');
title('x_1: Body Tilt Angle');

subplot(1,3,2);
plot(t_out, x_out(:,2), 'r-', 'LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('\dot{\theta} (rad/s)');
title('x_2: Body Angular Velocity');

subplot(1,3,3);
plot(t_out, x_out(:,3), 'g-', 'LineWidth',2); grid on;
xlabel('Time (s)'); ylabel('\dot{\phi} (rad/s)');
title('x_3: Wheel Angular Velocity');

sgtitle('Closed-Loop State Trajectories — Initial 1° Tilt', 'FontSize',13,'FontWeight','bold');

fprintf('All simulations complete! Check figures for results.\n');
