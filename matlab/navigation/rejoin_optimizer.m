function [rejoin_idx, rejoin_cost] = rejoin_optimizer(bot_pos_ENU, waypoints_ENU, current_wp_idx, lambda)
% REJOIN_OPTIMIZER  Find optimal re-entry waypoint after attack mode
%
%   After the bot collects trash and wants to return to lawnmower mode,
%   this function finds the best waypoint to resume from, minimising:
%
%     cost(j) = dist(bot_pos, wp_j) + lambda * (j - current_wp_idx)²
%
%   where:
%     - dist term : travel cost to reach waypoint j
%     - penalty term : deviation from last progress (avoids backtracking)
%
%   The optimal lambda balances time efficiency vs coverage completeness.
%
% Inputs:
%   bot_pos_ENU  : [E, N] bot position in local ENU metres
%   waypoints_ENU: Mx2 [E, N] all lawnmower waypoints
%   current_wp_idx: index of last completed waypoint (1-based)
%   lambda       : cost weight for deviation from progress (default 0.5)
%
% Outputs:
%   rejoin_idx   : index of optimal re-entry waypoint (>= current_wp_idx)
%   rejoin_cost  : cost value at optimal waypoint

if nargin < 4, lambda = 0.5; end

n_wps = size(waypoints_ENU, 1);
if current_wp_idx >= n_wps
    rejoin_idx  = n_wps;
    rejoin_cost = 0;
    return;
end

% Only consider waypoints at or ahead of current progress
candidates = current_wp_idx : n_wps;

costs = zeros(length(candidates), 1);
for k = 1:length(candidates)
    j = candidates(k);
    wp = waypoints_ENU(j,:);

    dist_cost  = norm(bot_pos_ENU - wp);
    prog_pen   = lambda * (j - current_wp_idx)^2;

    costs(k) = dist_cost + prog_pen;
end

[rejoin_cost, k_opt] = min(costs);
rejoin_idx = candidates(k_opt);

fprintf('[Re-join] Current WP: %d → Optimal re-join: %d (cost: %.2f m)\n', ...
    current_wp_idx, rejoin_idx, rejoin_cost);
end
