%% TEST_DUBINS.M  Unit tests for Dubins path planner
%
%  Run from matlab/ directory: >> run('tests/test_dubins.m')

addpath('../navigation');

fprintf('\n=== test_dubins ===\n');

R_MIN = 1.25;   % minimum turn radius [m]

%% Test 1: Path returns valid type
q0 = [0, 0, 0];
q1 = [5, 5, pi/4];
path = dubins_path(q0, q1, R_MIN);

valid_types = {'LSL','RSR','LSR','RSL'};
assert(ismember(path.type, valid_types), ...
    sprintf('FAIL T1: invalid path type "%s"', path.type));
fprintf('  [PASS] T1 — path type: %s\n', path.type);

%% Test 2: Path length > direct distance
direct = sqrt(5^2+5^2);
assert(path.length >= direct, 'FAIL T2: Dubins path shorter than straight line');
fprintf('  [PASS] T2 — length %.3f m >= %.3f m direct\n', path.length, direct);

%% Test 3: Path length < some reasonable upper bound
% For 5√2 ≈ 7.07m direct, Dubins should be < 20m
assert(path.length < 20, sprintf('FAIL T3: path too long %.3f m', path.length));
fprintf('  [PASS] T3 — path length %.3f m < 20m bound\n', path.length);

%% Test 4: Waypoints start near q0 and end near q1
wps = path.waypoints;
assert(~isempty(wps), 'FAIL T4: no waypoints generated');
start_err = norm(wps(1,1:2) - q0(1:2));
end_err   = norm(wps(end,1:2) - q1(1:2));
assert(start_err < 0.1, sprintf('FAIL T4: start error %.3f m', start_err));
assert(end_err   < 0.3, sprintf('FAIL T4: end error %.3f m', end_err));
fprintf('  [PASS] T4 — start err=%.3fm, end err=%.3fm\n', start_err, end_err);

%% Test 5: Straight-line test (q0 and q1 aligned, same heading)
q2 = [0,   0, pi/2];   % facing East
q3 = [5,   0, pi/2];   % 5m East, same heading
path2 = dubins_path(q2, q3, R_MIN);
assert(path2.length < 5.5, ...
    sprintf('FAIL T5: straight path %.3f m > 5.5m', path2.length));
fprintf('  [PASS] T5 — straight path length %.3f m ≈ 5m\n', path2.length);

%% Test 6: Rejoin optimizer returns index >= start
addpath('../navigation');
wps_ENU = rand(20, 2) * 10;
bot_pos = [3, 3];
[idx, cost] = rejoin_optimizer(bot_pos, wps_ENU, 8, 0.5);
assert(idx >= 8 && idx <= 20, sprintf('FAIL T6: rejoin idx %d out of range', idx));
assert(cost >= 0, 'FAIL T6: negative cost');
fprintf('  [PASS] T6 — rejoin idx=%d (must be >=8), cost=%.2f\n', idx, cost);

fprintf('\n=== test_dubins PASSED ===\n\n');
