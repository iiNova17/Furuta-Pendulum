function cfg = harness_config(mode)
%HARNESS_CONFIG Every rig-side setting of the harness, in one place: edit here.
%   cfg = harness_config()          'realistic': sensors and firmware behave as on the rig
%   cfg = harness_config('ideal')   perfect sensors and actuator (first design steps only)
%   Per run, without editing this file:  cfg = harness_config(); cfg.Ts = 2e-3; run_sim(..., 'Cfg', cfg)
%
%   The plant parameters are NOT set here: they are read from matlab/model/furuta_params.m,
%   the identified model of the report (Table 6). Change the model there, never here.
%   Controller gains are not here either: they live in the controller's design function
%   (e.g. controllers/hybrid/ctrl_hybrid_design.m).
arguments
    mode (1,:) char {mustBeMember(mode, {'realistic', 'ideal'})} = 'realistic'
end
[cfg.p, cfg.lin, cfg.breakaway] = load_identified_model();   % breakaway: measured arm stiction, for the controller design
cfg.mode = mode;

% timing: three rates
cfg.Ts = 4e-3;     % CONTROLLER period [s]: Simulink + Stateflow on the PC, HIL over USB (brief sec. 5).
                   % 4 ms = the firmware's 250 Hz stream; set it to what the HIL link achieves.
cfg.fw.Ts = 1e-3;  % FIRMWARE loop on the ESP32 (slew, standby, trips) and sensor sampling [s]
cfg.dt = 5e-4;     % PLANT integration step [s], fixed-step RK4 (the report validated with RK4 at 2 ms)

% HIL link between the ESP32 and the controller (USB serial). Unknown until measured: 0 = none.
cfg.link.y_delay = 0;   % extra controller periods before a measurement reaches the controller
cfg.link.u_delay = 0;   % extra controller periods before a duty command reaches the firmware

% which rig the controller talks to: 'Simulation' (this model) or 'Hardware' (the ESP32, HIL)
cfg.rig = 'Simulation';

% performance metrics (brief sec. 4.7, see harness_metrics)
cfg.metrics.band_th1 = 1;      % theta1 settling / recovery band [deg]
cfg.metrics.band_th0 = 5;      % theta0 step settling band [% of the step]
cfg.metrics.ss_window = 1;     % steady-state window at the end of a run [s]

% sensors (report Table 1 and section 5.2)
s.arm_cpr = 1496;        % arm encoder counts/rev: 11 PPR x4 on the motor, 34:1 gearbox
s.pend_cpr = 4096;       % AS5600, 12 bit
s.pend_period = 2e-3;    % AS5600 read at 500 Hz
s.pend_delay = 7e-3;     % pendulum angle lags the arm and current by t_d = 7 ms (report 5.2.1)
s.i_noise_sd = 0.05;     % current noise before the MCU filter [A] -- ASSUMED, not measured
s.i_fc = 40;             % MCU current filter: 2nd-order Butterworth, 40 Hz (report Table 1)
s.quantize = true;
s.i_filter = true;
cfg.sens = s;

% firmware actuator and safety (firmware/include/engine.h, report 5.1)
a.max_duty = 0.5;          % duty limit used for every powered experiment in the report
a.slew = 100;              % duty per second. 100 is the firmware maximum; the profile default 0.5/s
                           % is for open-loop tests and is far too slow for feedback -- raise it on the rig
a.standby = 1e-3;          % the duty passes through zero (TB6612 standby) for one tick when reversing
a.duty_levels = 1023;      % 10-bit PWM
a.travel = deg2rad(120);   % ARM_TRAVEL trip, measured arm angle from centre
a.i_trip = 1.5;            % OVERCURRENT trip on the filtered current [A]
a.i_trip_time = 10e-3;     % ... after this long above it [s]
a.reverse_through_zero = true;   % false only to replay a recorded applied duty unchanged
cfg.act = a;

if strcmp(mode, 'ideal')
    cfg.sens.quantize = false;
    cfg.sens.pend_delay = 0;
    cfg.sens.pend_period = cfg.fw.Ts;
    cfg.sens.i_noise_sd = 0;
    cfg.sens.i_filter = false;
    cfg.act.slew = Inf;
    cfg.act.standby = 0;
    cfg.act.duty_levels = Inf;
end
check_rates(cfg);
end

function check_rates(cfg)
% The three rates must nest: Ts a multiple of fw.Ts, fw.Ts a multiple of dt.
isMult = @(a, b) abs(a/b - round(a/b)) < 1e-9 && round(a/b) >= 1;
assert(isMult(cfg.Ts, cfg.fw.Ts) && isMult(cfg.fw.Ts, cfg.dt), 'harness_config:rates', ...
    'cfg.Ts (%g) must be a multiple of cfg.fw.Ts (%g), which must be a multiple of cfg.dt (%g)', ...
    cfg.Ts, cfg.fw.Ts, cfg.dt);
end

function [p, lin, breakaway] = load_identified_model()
% Run furuta_params.m in an isolated workspace and keep the struct p and the linear models.
here = fileparts(mfilename('fullpath'));
S = run_script(fullfile(here, '..', 'model', 'furuta_params.m'));
p = S.p;
breakaway = struct('tsp', p.tsp, 'tsn', p.tsn);   % the controllers' motor inverse and friction compensation use it
% The Simscape plant has no arm breakaway (stick-slip) friction yet, so the harness uses the model without it
% to keep the Simscape plant equal to furuta_rhs (tests/test_harness.m, validate_open_loop.m). Remove these two
% lines once the plant's friction block does the same as furuta_rhs: docs/HANDOFF_BREAKAWAY.md.
p.tsp = 0;
p.tsn = 0;
lin = struct('A_up', S.A_up, 'B_up', S.B_up, 'Bd_up', S.Bd_up, ...
             'A_down', S.A_down, 'B_down', S.B_down, 'Bd_down', S.Bd_down, ...
             'C', S.C_up, 'D', S.D_up);
end

function S = run_script(file)
evalc('run(file)');            % evalc: furuta_params prints minreal messages
names = setdiff(who, {'file'});
for k = 1:numel(names)
    S.(names{k}) = eval(names{k});
end
end
