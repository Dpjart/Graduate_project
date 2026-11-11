function c = rov_config()
%ROV_CONFIG  Experiment settings for run_experiments (ROV counterpart of uav_config).
%   Edit here: which thruster layouts to compare, experiment timing,
%   disturbance strength, initial offset and failure case.
%   SI units. NED frame (z down = depth).

c.p = rov_params();

%% Thruster layouts to compare (tag -> folder name, builder)
c.z_off = 0;                       % all thrusters at CoG height (-0.10 = top-mounted)
c.layouts = { ...
    struct('tag', '4',    'build', @(p) cfg_4(p, c.z_off)), ...
    struct('tag', '5',    'build', @(p) cfg_5(p, c.z_off)), ...
    struct('tag', '6_45', 'build', @(p) cfg_6(p, 45, c.z_off)) };
% Alpha sweep: append e.g.
%   struct('tag','6_30','build',@(p) cfg_6(p,30,c.z_off))
%   struct('tag','6_60','build',@(p) cfg_6(p,60,c.z_off))

%% Controller (identical for every layout - fairness)
c.mpc_opts = struct();             % defaults in mpc_setup.m

%% Experiments (same structure as the UAV experiment design)
%  1 line            straight-line tracking, no disturbance
%  2 helix           complex 3D path, no disturbance
%  3 helix+current   side current switched on during c.disturbanceWindow
%  4 helix+offset    start away from the path with a heading error
%  5 helix+failure   thruster f1 fails at c.failTime (MPC informed)
c.experimentNames = {'Straight line', 'Helix', 'Helix with current', ...
                     'Initial offset', 'Thruster failure'};
c.manoeuvres      = {'line', 'helix', 'helix', 'helix', 'helix'};
c.duration        = 35;            % [s] every experiment
c.disturbanceWindow = [8 14];      % [s] current active on [start, end)
c.current         = [0; 0.2; 0];   % [m/s] NED current while active (east = sideways)
c.initialOffset   = [0.6; -0.5; 0.3];   % [m] experiment 4 position offset
c.initialYawOffset = 0.5;          % [rad] experiment 4 heading offset
c.failThruster    = 1;             % experiment 5 failed thruster index
c.failTime        = 10;            % [s]

%% Simulation and metrics
c.tether            = false;           % no tether: free-swimming vehicle
c.thruster_lag      = true;
c.dt_sim            = 0.01;
c.recoveryTolerance = 0.05;        % [m] absolute 3D error
c.recoveryDwell     = 1;           % [s] continuously inside tolerance
c.outputDir = fullfile(fileparts(mfilename('fullpath')), 'results');
end
