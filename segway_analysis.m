%% =========================================================================
%  segway_analysis.m
%  Controllability, Observability, and Stability Analysis
%
%  Requires: segway_model.m to be run first (defines A, B, C, D, sys_ol)
% =========================================================================

run('segway_model.m');

fprintf('=== SYSTEM ANALYSIS ===\n\n');

%% Controllability
% The controllability matrix is C = [B  AB  A²B]
% If rank = n (= 3), we can place poles ANYWHERE using state feedback u = -Kx
Ctrl = [B, A*B, A^2*B];
rank_ctrl = rank(Ctrl);

fprintf('1. CONTROLLABILITY\n');
fprintf('   Controllability matrix [B  AB  A²B]:\n');
disp(Ctrl);
fprintf('   Rank = %d / 3\n', rank_ctrl);
if rank_ctrl == 3
    fprintf('   --> FULLY CONTROLLABLE ✓\n');
    fprintf('   --> Any closed-loop pole placement is achievable\n\n');
else
    fprintf('   --> NOT CONTROLLABLE ✗\n\n');
end

%% Observability
% The observability matrix is O = [C; CA; CA²]
% If rank = n (= 3), all internal states can be inferred from output y = θ
Obs = [C; C*A; C*A^2];
rank_obs = rank(Obs);

fprintf('2. OBSERVABILITY\n');
fprintf('   Observability matrix [C; CA; CA²]:\n');
disp(Obs);
fprintf('   Rank = %d / 3\n', rank_obs);
if rank_obs == 3
    fprintf('   --> FULLY OBSERVABLE ✓\n');
    fprintf('   --> All states can be reconstructed from y = θ alone\n\n');
else
    fprintf('   --> NOT OBSERVABLE ✗\n\n');
end

%% Lyapunov Stability
% Requires ALL eigenvalues of A to have strictly negative real parts
ev = eig(A);
fprintf('3. LYAPUNOV STABILITY\n');
fprintf('   Open-loop eigenvalues of A:\n');
for i = 1:length(ev)
    flag = '';
    if real(ev(i)) > 0, flag = '  <-- UNSTABLE'; end
    fprintf('   λ%d = %+.8f %+.8fj%s\n', i, real(ev(i)), imag(ev(i)), flag);
end
if all(real(ev) < 0)
    fprintf('   --> Lyapunov STABLE ✓\n\n');
else
    fprintf('   --> NOT Lyapunov stable ✗ (saddle-focus: robot tips over!)\n\n');
end

%% BIBO Stability
[~, den] = tfdata(sys_ol, 'v');
poles_tf = roots(den);
fprintf('4. BIBO STABILITY\n');
fprintf('   Transfer function poles:\n');
for i = 1:length(poles_tf)
    flag = '';
    if real(poles_tf(i)) > 0, flag = '  <-- RHP pole!'; end
    fprintf('   s%d = %+.8f %+.8fj%s\n', i, real(poles_tf(i)), imag(poles_tf(i)), flag);
end
if all(real(poles_tf) < 0)
    fprintf('   --> BIBO STABLE ✓\n\n');
else
    fprintf('   --> NOT BIBO stable ✗\n\n');
end

%% Characteristic Polynomial
fprintf('5. CHARACTERISTIC POLYNOMIAL\n');
fprintf('   Δ(s) = s³ + %.6f s² + %.6f s + %.6f\n\n', ...
        den(2), den(3), den(4));

fprintf('SUMMARY: Open-loop system is UNSTABLE (saddle-focus).\n');
fprintf('State feedback control is required to stabilize the Segway.\n');
