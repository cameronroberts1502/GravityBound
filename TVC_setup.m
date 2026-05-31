%% ========================================================================
%  TVC PROJECT -- control setup   (the analog of AAP5_setup.m)
%
%  Builds the plant transfer functions, then you tune interactively in
%  Control System Designer (SISOtool) -- exactly like Tasks 4 & 6.
%
%  Inner loop  : gimbal deflection (delta) -> tilt angle      [Task 4 analog]
%  Outer loop  : tilt -> horizontal position                  [Task 6 analog]
%
%  Requires the Control System Toolbox.
% =========================================================================
close all; clear;

%% --- Physical parameters (MEASURE / ESTIMATE FOR YOUR RIG) --------------
m     = 0.5;     % vehicle mass                              [kg]
g     = 9.81;
J     = 0.008;   % inertia about CoM (~ about pivot too)     [kg m^2]
b     = 0.001;   % rotational damping                        [N m s]
l_t   = 0.20;    % Stage 1: pivot -> thrust point            [m]
l_cm  = 0.10;    % Stage 1: pivot -> CoM (sets instability)  [m]
l     = 0.10;    % Stage 2: CoM -> thrust point              [m]
tau_s = 0.05;    % servo first-order lag                     [s]

T = m*g;         % operating-point thrust (hover)            [N]
s = tf('s');

%% --- INNER attitude plant:  delta -> tilt  ------------------------------
% STAGE 1 (tethered, fixed pivot): inverted pendulum  ->  RHP pole (unstable)
P_att_S1 = (T*l_t) / (J*s^2 + b*s - m*g*l_cm) * 1/(tau_s*s + 1);

% STAGE 2 (free flight): gravity acts THROUGH the CoM -> no destabilising
% term -> double integrator (marginally stable)
P_att_S2 = (T*l)   / (J*s^2 + b*s)            * 1/(tau_s*s + 1);

% PARAMETERISED version: slide kg from m*g*l_cm (Stage 1) down to 0 (Stage 2)
% to verify ONE tuning is robust to the plant change  [LO 2.6: param variation]
make_Patt = @(kg) (T*l_t)/(J*s^2 + b*s - kg) * 1/(tau_s*s + 1);

%% --- OUTER position plant (Stage 2 only):  tilt -> position -------------
% xddot = (T/m)*sin(theta) ~= (T/m)*theta   ->   x/theta = (T/m)/s^2
P_pos = (T/m)/s^2;

%% --- Spec  ->  s-plane target region  (edit to your spec) --------------
OS = 0.20;            % max overshoot (fraction)     [Task 4 used 20%]
Tp = 0.20;            % max peak time (s)            [Task 4 used 0.2 s]
zeta_min = -log(OS)/sqrt(pi^2 + log(OS)^2);
wd_min   = pi/Tp;
fprintf('\nTarget dominant-pole region:  zeta >= %.3f ,  wd >= %.1f rad/s\n', ...
        zeta_min, wd_min);
fprintf('Enter these in Control System Designer as design requirements.\n');
fprintf('Open-loop poles (Stage 1):\n'); disp(pole(P_att_S1))

%% --- NEXT STEPS (interactive -- mirrors your assignment) ----------------
% TASK-4 analog (inner loop, design on the harder Stage-1 plant):
%     controlSystemDesigner(P_att_S1)
%   - right-click plot -> Design Requirements: set % overshoot = 20, and a
%     settling/peak requirement matching wd_min above
%   - add a PID with TWO REAL ZEROS; drag gain so poles enter the region
%   - Export the compensator as  C_inner
%
% ROBUSTNESS check (one tuning, both stages):
%     figure; hold on; grid on; sgrid(zeta_min,[])
%     for kg = linspace(m*g*l_cm, 0, 5)
%         pzmap(feedback(C_inner*make_Patt(kg), 1));
%     end
%     title('Closed-loop poles: fixed rig -> free flight (kg -> 0)');
%   -> confirm every marker stays left of the zeta_min ray.
%
% TASK-6 analog (outer loop, Stage 2 -- inner left UNTOUCHED):
%     Tin    = feedback(C_inner*P_att_S2, 1);   % closed inner loop
%     P_outer = Tin * P_pos;                     % tilt-ref -> position
%     controlSystemDesigner(P_outer)            % design a PD (+ LP filter)
%   - keep the outer loop ~5x slower than the inner so they don't fight.

% controlSystemDesigner(P_att_S1)   % <- uncomment to start