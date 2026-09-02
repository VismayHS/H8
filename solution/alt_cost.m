function J = alt_cost(gains, P, z_ref, Tsim, opt)
%ALT_COST  Scalar cost for PID gain optimisation (Task 2, Method D).
%
%   J = ALT_COST(gains, P, z_ref, Tsim, opt)
%
%   Cost = ITAE, plus penalties on overshoot, control effort and control
%   activity. ITAE is the standard choice because the time weighting
%   punishes errors that persist, which is exactly what you want for a
%   tracking task, while tolerating the unavoidable error right after a step.
%
%   Tune the weights below if the optimiser converges somewhere you dislike:
%     - raise w_OS  to trade speed for a gentler response
%     - raise w_U   if the thrust command keeps hitting saturation
%     - raise w_dU  to discourage chattering

if nargin < 5, opt = struct(); end

w_OS  = 0.30;     % weight on percent overshoot
%  RAISED from 0.05. At the old weight, ITAE dominated and the optimiser
%  happily bought a faster settling time with enormous overshoot - the
%  learned controller flew to 1.62 m on a 1.00 m command. For an altitude
%  controller overshoot is a collision risk, not a cosmetic defect, so it
%  must cost real money in the objective.
w_U   = 0.02;     % weight on peak thrust above hover
w_dU  = 0.002;    % weight on total control variation
w_reg = 2e-4;     % regularisation on gain magnitude
%
%  WHY REGULARISE: without it the optimiser drives Kp into the thousands to
%  brute-force a steady-state error that integral action should be handling.
%  Such gains are physically useless - they amplify sensor noise, chatter the
%  actuator, and sit permanently in saturation. The penalty below expresses
%  the real engineering constraint and pushes the optimiser towards using Ki
%  for steady-state error, which is what it is for.

try
    R = sim_altitude(gains, P, z_ref, Tsim, opt);
catch
    J = 1e6; return;
end

% Reject anything that diverged or went non-finite
if any(~isfinite(R.z)) || max(abs(R.z)) > 50*max(abs(z_ref),1)
    J = 1e6; return;
end

M = perf_metrics(R.t, R.z, z_ref, R.U1);

J = M.ITAE ...
  + w_OS  * M.OS ...
  + w_U   * max(0, M.Umax - P.W) ...
  + w_dU  * M.dU ...
  + w_reg * (gains(1)^2 + gains(3)^2);   % penalise huge Kp / Kd

% Hard penalty if it never settles
if isnan(M.Ts), J = J + 50; end

if ~isfinite(J), J = 1e6; end
end
