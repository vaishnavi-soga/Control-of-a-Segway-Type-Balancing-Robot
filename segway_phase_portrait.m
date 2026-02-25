%% =========================================================================
%  segway_phase_portrait.m
%  3D Phase Portrait — Open-Loop System
%
%  Plots trajectories in [θ, θ̇, φ̇] state space from various ICs.
%  Result: Outward spiraling trajectories = open-loop system is UNSTABLE.
%
%  The saddle-focus structure comes from:
%    λ1 ≈ -7.04          → one stable (contracting) direction
%    λ2,3 ≈ 1.13 ± 4.58j → two unstable (spiraling) directions
%
%  Requires: segway_model.m to be run first
% =========================================================================

run('segway_model.m');

fprintf('Generating 3D Phase Portrait (may take ~10 seconds)...\n');

f_ol  = @(t,x) A*x;          % Open-loop: no control input
tspan = linspace(0, 3, 400);  % Simulate for 3 seconds

% Grid of initial conditions
theta_ic    = linspace(-0.18, 0.18, 6);   % Initial tilt angles
thetadot_ic = linspace(-0.8,  0.8,  6);   % Initial angular velocities

figure('Name','3D Phase Portrait','Position',[100 80 800 600]);
hold on; grid on;

xlabel('\theta (rad)', 'FontSize',12);
ylabel('\dot{\theta} (rad/s)', 'FontSize',12);
zlabel('\dot{\phi} (rad/s)', 'FontSize',12);
title({'3D Phase Portrait — Open-Loop Segway System', ...
       'Outward spirals confirm the system is inherently UNSTABLE'}, 'FontSize',12);
view(42, 28);
set(gca, 'FontSize', 11, 'LineWidth', 1.1);

cmap = parula(length(tspan));  % Color by time: blue=start, yellow=end

for th0 = theta_ic
    for dth0 = thetadot_ic
        x0 = [th0; dth0; 0];
        [~, x] = ode45(f_ol, tspan, x0);

        % Plot with time-colored segments
        for k = 1:length(tspan)-1
            plot3(x(k:k+1,1), x(k:k+1,2), x(k:k+1,3), ...
                  'Color', cmap(k,:), 'LineWidth', 1.6);
        end

        % Direction arrows
        arrow_idx = round(linspace(1, length(tspan)-1, 3));
        for ai = arrow_idx
            quiver3(x(ai,1), x(ai,2), x(ai,3), ...
                    x(ai+1,1)-x(ai,1), x(ai+1,2)-x(ai,2), x(ai+1,3)-x(ai,3), ...
                    0, 'k', 'MaxHeadSize', 1.2);
        end
    end
end

% Mark equilibrium point at origin
plot3(0,0,0,'ko','MarkerFaceColor','k','MarkerSize',8);
text(0.01, 0.05, 0, 'Equilibrium (unstable)', 'FontSize',9, 'Color','k');

colormap(parula);
colorbar('Ticks',[0 1],'TickLabels',{'t=0','t=end'});
hold off;

fprintf('Phase portrait complete.\n');
fprintf('Key insight: trajectories spiral OUTWARD → robot falls without control.\n');
