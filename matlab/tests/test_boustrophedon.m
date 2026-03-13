%% TEST_BOUSTROPHEDON.M  Unit tests for boustrophedon path planner
%
%  Run from matlab/ directory: >> run('tests/test_boustrophedon.m')

addpath('..');
addpath('../navigation');

fprintf('\n=== test_boustrophedon ===\n');
errors = 0;

%% Test 1: Rectangular polygon → correct number of rows
REF = [13.0827, 77.5946];
% 15m × 10m rectangle in GPS
poly_GPS = zeros(4,2);
m_lat = 111320; m_lon = 111320*cos(deg2rad(REF(1)));
poly_ENU = [0,0; 15,0; 15,10; 0,10];
poly_GPS(:,1) = REF(1) + poly_ENU(:,2)/m_lat;
poly_GPS(:,2) = REF(2) + poly_ENU(:,1)/m_lon;

sweep = 0.6;
[wps_ENU, wps_GPS, sw, n_rows] = boustrophedon(poly_GPS, REF, sweep);

expected_rows = ceil(10/sweep);
assert(abs(n_rows - expected_rows) <= 1, ...
    sprintf('FAIL T1: expected %d rows, got %d', expected_rows, n_rows));
fprintf('  [PASS] T1 — row count: expected %d, got %d\n', expected_rows, n_rows);

%% Test 2: Waypoints are inside polygon
% Every waypoint should have 0 < E < 15, 0 < N < 10
for i=1:size(wps_ENU,1)
    E = wps_ENU(i,1); N = wps_ENU(i,2);
    assert(E >= -0.1 && E <= 15.1 && N >= -0.1 && N <= 10.1, ...
        sprintf('FAIL T2: waypoint %d [%.2f, %.2f] outside polygon', i, E, N));
end
fprintf('  [PASS] T2 — all %d waypoints inside polygon\n', size(wps_ENU,1));

%% Test 3: GPS round-trip (ENU→GPS→ENU should be < 1mm error)
for i=1:min(5, size(wps_ENU,1))
    lat = wps_GPS(i,1); lon = wps_GPS(i,2);
    E_rt = (lon - REF(2)) * m_lon;
    N_rt = (lat - REF(1)) * m_lat;
    err_m = sqrt((E_rt-wps_ENU(i,1))^2 + (N_rt-wps_ENU(i,2))^2);
    assert(err_m < 0.001, sprintf('FAIL T3: GPS round-trip error %.4f m', err_m));
end
fprintf('  [PASS] T3 — GPS projection round-trip error < 1mm\n');

%% Test 4: Path alternates direction (boustrophedon pattern)
% Check that consecutive row start/end x alternates
if size(wps_ENU,1) >= 4
    x1 = wps_ENU(1,1); x2 = wps_ENU(2,1);  % row 1: left→right
    x3 = wps_ENU(3,1); x4 = wps_ENU(4,1);  % row 2: should be right→left
    dir1 = sign(x2 - x1);
    dir2 = sign(x4 - x3);
    assert(dir1 ~= dir2 || dir1==0, 'FAIL T4: boustrophedon direction not alternating');
    fprintf('  [PASS] T4 — boustrophedon direction alternates correctly\n');
end

%% Test 5: Sweep width respected
if n_rows >= 2
    % Y-coordinates of first two start points should be ~sweep apart
    y1 = wps_ENU(1,2);
    y3 = wps_ENU(3,2);
    dy = abs(y3 - y1);
    assert(abs(dy - sweep) < 0.01, sprintf('FAIL T5: row spacing %.4f ≠ %.4f', dy, sweep));
    fprintf('  [PASS] T5 — row spacing = %.4f m (expected %.4f)\n', dy, sweep);
end

fprintf('\n=== test_boustrophedon PASSED (%d errors) ===\n\n', errors);
