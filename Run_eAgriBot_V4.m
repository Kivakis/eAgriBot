%%

clear; close all; clc;

%% Selectable parameters
% Select robot model, soil type, and main cycle

% Five models to select from:
% 1: Rear-wheel drive (RWD), 40/60 weight distribution
% 2: RWD, 30/70 weight distribution
% 3: Front-wheel drive (FWD), 60/40 weight distribution
% 4: FWD, 70/30 weight distribution
% 5: Four-wheel drive (4WD), 50/50 weight distribution

model = 2;

% Two soil types to select from:
% 1: sandy loam
% 2: clayey loam

soil_type = 1;


% Cycle definition

% The main cycle contains acceleration, constant speed, and deceleration
% sections.

% The robot first spawns into the world and performs a short cycle to
% ensure the rear tires are already in the ruts caused by the front tires
% when the main cycle begins.
total_dist_target = 100; % m, distance after which decel. begins in main cycle
vel_max = 8; % km/h, target speed for constant speed section of main cycle
accel = 0.1*9.81; % m/s^2, target accel. in main cycle
decel = 0.15*9.81; % m/s^2 target decel. in main cycle

Har_tine_depth = 0.0740; % m, set value for tine depth
% The tines will end up a bit deeper due to harrow tire sinkage

% Multiplier for rear control value with 4WD. Default 1
% Increase value to increase rear motor torque relative to front axle
% torque (Sensible values: 1-1.5)
T_multi_r = 1.5;

step_size = 1e-3; % Simulation step size

%% Key parameter definitions

if model == 1
    drivetype = 1; % 1 = RWD, 2 = FWD, 3 = 4WD
    weight_bias = 0.4; % Front load fraction (static)
    brake_bias = 0.0; % Front brake fraction
elseif model == 2
    drivetype = 1;
    weight_bias = 0.3;
    brake_bias = 0.0;
elseif model == 3
    drivetype = 2;
    weight_bias = 0.6;
    brake_bias = 1.0;
elseif model == 4
    drivetype = 2;
    weight_bias = 0.7;
    brake_bias = 1.0;
else
    drivetype = 3;
    weight_bias = 0.5;
    brake_bias = 0.5;
end

% simu_max_time = 120; % Maximum simulation time

i_rear = 17.9542; % Rear gear ratio
i_front = 17.9542; % Front gear ratio
gb_eff = 0.98; % Gear efficiency

if drivetype < 3
    m = 490; % Total mass of the vehicle (kg), default 450 kg
else
    m = 520; % Total mass with 4WD
end
m_tool = 200; % Towed tool mass (kg)
g = 9.81; % Standard accel. due to gravity (m/s^2)
Fb_max = m*g; % Maximum brake force (N), default m*g

J_drive = 15; % Total inertia at driven axle (including tire inertias), default 15
J_nondrive = 6; % Total inertia at non-driven axle (including tire inertias), default 6

% Control gain values
P_gain = 1.0;
I_gain = 0.0;

SOC_init = 0.8; % Initial battery SOC

if drivetype == 1
    brake_bias = 0;
    J_rear = J_drive;
    J_front = J_nondrive;
elseif drivetype == 2
    brake_bias = 1;
    J_rear = J_nondrive;
    J_front = J_drive;
else
    J_rear = J_drive;
    J_front = J_drive;
end

%% Define slip limits for traction control for each model and soil type

slip_trig = 0.02; % How much before approximated optimal slip ratio the TC starts to intervene

if model == 1 % RWD, sr_lim_slip_f has no effect
    if soil_type == 1
        sr_lim_slip_f = 0.26-slip_trig;
        sr_lim_slip_r = 0.26-slip_trig;
    else
        sr_lim_slip_f = 0.12-slip_trig;
        sr_lim_slip_r = 0.12-slip_trig;
    end
elseif model == 2 % RWD, sr_lim_slip_f has no effect
    if soil_type == 1
        sr_lim_slip_f = 0.24-slip_trig;
        sr_lim_slip_r = 0.24-slip_trig;
    else
        sr_lim_slip_f = 0.11-slip_trig;
        sr_lim_slip_r = 0.11-slip_trig;
    end
elseif model == 3 % FWD, sr_lim_slip_r has no effect
    if soil_type == 1
        sr_lim_slip_f = 0.30-slip_trig;
        sr_lim_slip_r = 0.30-slip_trig;
    else
        sr_lim_slip_f = 0.17-slip_trig;
        sr_lim_slip_r = 0.17-slip_trig;
    end
elseif model == 4 % FWD, sr_lim_slip_r has no effect
    if soil_type == 1
        sr_lim_slip_f = 0.28-slip_trig;
        sr_lim_slip_r = 0.28-slip_trig;
    else
        sr_lim_slip_f = 0.15-slip_trig;
        sr_lim_slip_r = 0.15-slip_trig;
    end
else % 4WD
    if soil_type == 1
        sr_lim_slip_f = 0.33-slip_trig;
        sr_lim_slip_r = 0.27-slip_trig;
    else
        sr_lim_slip_f = 0.20-slip_trig;
        sr_lim_slip_r = 0.14-slip_trig;
    end
end

sr_lim_skid = -0.2; % Slip ratio above which torque is limited, default -0.2

%% Tire-soil parameters

% Sandy loam, 50% moisture
if soil_type == 2
    c = 3300;
    k_c = 24.45;
    k_phi = 96.34;
    phi = 33.7; % deg
    k_xy = 0.0076; % m, 0.0076
    gamma_s = 15196; % N/m^3
    
    rho_t = gamma_s/9.81;
    mu = 0.35;
    
    R_u = 0.635/2; % m (tire radius)
    wid = 0.2032; % m (tire width)
    c_0 = 0.4;
    c_1 = 0.2;
    c_0f = 0.2;
    c_1f = 0.05;
    n_0 = 0.8;
    n_1 = 0.6;
    k_1 = 0.1178;
    k_2 = 0.1672;
    k_3 = 0.0348;
    t_params = [c k_c k_phi phi k_xy gamma_s R_u wid c_0 c_1 c_0f c_1f n_0 n_1 k_1 k_2 k_3];

% Clayey loam, 47% moisture
else
    c = 6100;
    k_c = 4.43;
    k_phi = 87.60;
    phi = 26.6; % deg
    k_xy = 0.0037; % m, 0.0076
    gamma_s = 16324; % N/m^3
    
    rho_t = gamma_s/9.81;
    mu = 0.35;
    
    R_u = 0.635/2; % m (tire radius)
    wid = 0.2032; % m (tire width)
    c_0 = 0.4;
    c_1 = 0.2;
    c_0f = 0.2;
    c_1f = 0.05;
    n_0 = 0.8;
    n_1 = 0.6;
    k_1 = 0.1178;
    k_2 = 0.1672;
    k_3 = 0.0348;
    t_params = [c k_c k_phi phi k_xy gamma_s R_u wid c_0 c_1 c_0f c_1f n_0 n_1 k_1 k_2 k_3];
end

exit_angle = -1;
reso = 2; % Angle resolution, number of segments per one degree

theta_m_max = 0.9; % Maximum percentage of theta_e allowed for theta_m
% Allowing this to go too high may result in issues with some
% calculations outputting complex numbers.

s_d_tol = 0.1;

po = 3;

params2 = [exit_angle reso theta_m_max s_d_tol po];

ground_res = 0.01; % Ground resolution, default 1 cm (0.01)

%% Vehicle parameters

P_aux = 200;

h_CoG = 0.6;
l = 1.29;
% l_f = 0.9;
l_f = (1-weight_bias)*l;

r_rear = R_u;
r_front = R_u;

RT_c_z_rel = R_u-h_CoG;
RT_c_x_rel = l_f-l;
FT_c_z_rel = R_u-h_CoG;
FT_c_x_rel = l_f;

Hook_z_rel = -0.3; % Hook height relative to CoG
Hook_x_rel = l_f-l-0.4; % Hook x-position relative to CoG

Tool_x_rel = -1.3; % Tool CoG x-position relative to the hook
Tool_z_rel = -0.08; % Tool CoG height relative to the hook

% Tool_bottom_x_rel = -1.6; % Tool bottom point x-position relative to the hook
% Tool_bottom_z_rel = abs(Hook_z_rel)-h_CoG; % Tool bottom point height relative to the hook

% Harrow geometry, two rows of tines

F_tines = 5;
B_tines = 5;

Tine_vz_damping = 0;
Har_tire_Fzmulti = 8e4;
Har_tire_Fzdamp = 8e3;

Har_angle = deg2rad(25);
Har_tine_length = 0.25; % m
Har_tine_width = 0.02;

Har_tire_rad = 0.5842/2; % 23x7 tire
Har_tire_wid = 0.1778; % 23x7 tire
J_harrow = 1; % Harrow axle inertia, 0.3 default
Har_tire_x_rel = -1.5; % Tire axis x-coordinate relative to hook
Har_tire_z_rel = abs(Hook_z_rel)-h_CoG+Har_tire_rad; % Tire axis z-coordinate relative to hook

t_params_har = [c k_c k_phi phi k_xy gamma_s Har_tire_rad Har_tire_wid c_0 c_1 c_0f c_1f n_0 n_1 k_1 k_2 k_3];

% Tine back row

Har_back_top_x_rel = -2.0; % Relative to hook
Har_back_tip_x_rel = Har_back_top_x_rel+Har_tine_length*cos(Har_angle);
Har_back_tip_z_rel = abs(Hook_z_rel)-h_CoG-Har_tine_depth;
Har_back_top_z_rel = Har_back_tip_z_rel+Har_tine_length*sin(Har_angle);

% Tine front row

Har_front_top_x_rel = -1.2; % Relative to hook
Har_front_tip_x_rel = Har_front_top_x_rel+Har_tine_length*cos(Har_angle);
Har_front_tip_z_rel = abs(Hook_z_rel)-h_CoG-Har_tine_depth;
Har_front_top_z_rel = Har_front_tip_z_rel+Har_tine_length*sin(Har_angle);

I_y = 150; % Rotational inertia of the vehicle (y-axis)

I_y_tool = 20; % Rotational inertia of the towed tool (y-axis)
F_res = 100; % Towed tool resistance force (N), not used
tool_support_gain = 8e3; % kg/s^2, multiplies tool depth to acquire F_z, not used
k_d_tool = 2e3; % Tool vertical damping coefficient (kg/s), not used

omega_tool_init = 0;
theta_tool_init = 0;

init_omega = 0; % Initial pitch rate (default 0 rad/s)
init_theta = 0; % Initial pitch (default 0 rad)

k_d = 50e2; % Tire-soil vertical force damping
k_d_har = 50e2; % Tire-soil vertical force damping for harrow tires
v_init = 0; % Vehicle CoG initial speed (x-axis)
s_init = 3; % Vehicle CoG initial position (x-axis)
init_CoG_v_z = 0; % Vehicle CoG initial vertical speed (z-axis)
init_CoG_height = h_CoG-0.001; % Vehicle CoG initial vertical position (z-axis)


% The calculations below are to determine the index_init value, which is
% the initial ground point of the rear-most point of the contact patch of
% the front tire (the value is used in the Delay block for "Index" in the
% Ground Properies Update subsystem in the Vehicle Dynamics subsystem.

FT_center_init = [cos(init_theta) sin(init_theta); -sin(init_theta) cos(init_theta)]*...
        [FT_c_x_rel; FT_c_z_rel]+[s_init; init_CoG_height];

FT_init_x = FT_center_init(1);

FT_init_exit_x = FT_init_x+R_u*sin(deg2rad(exit_angle));
index_init = round(FT_init_exit_x/ground_res)+1;


%

% Fz_rear_init = m*g*(1-weight_bias); % Initial rear axle load
% Fz_front_init = m*g*weight_bias; % Initial front axle load

% slip_ub = r_rear;
% slip_ub_har = Har_tire_rad;
slip_ub = r_rear*1.75; % 2.0 km/h with 0.3175 m wheel radius
slip_ub_har = Har_tire_rad*1.90;

%% Load motor and battery data

load('EM_data_V4.mat')
load('EM_effmap_V4.mat');
load('Battery_data_V5_Kokam_NMC.mat');

% Halve the motor torque for 4WD
if drivetype == 3
    T = T/2;
    T_effmap = T_effmap/2;
end

T = 2*T;
T_effmap = 2*T_effmap;

%%

% figure
% plot(w, T, '-b'), grid on
% xlabel('Speed (rad/s)'), ylabel('Torque (Nm)')
% ylim([0 max(T)+20])
% hold on;
% 
% contour(w_effmap, T_effmap, effmap', [0:0.1:0.8 0.85:0.05:1], 'ShowText', 'on');
% grid on;
% xlabel('Speed (rad/s)');

%% Initial speed and slip

% v_init = 0; % Initial speed of the vehicle
slip_init = 0;

%% Cycle definition

t_sim = 9999;

% Cycle definition

% Pre-cycle
pre_start = 5; % s, when the pre-cycle starts
pre_dist_target = 3; % m
pre_vel = 5; % km/h
pre_accel = 0.1*9.81; % m/s^2
pre_decel = 0.15*9.81; % m/s^2

pre_accel_time = pre_vel/3.6/pre_accel;
pre_accel_dist = 1/2*pre_accel*pre_accel_time^2;
pre_decel_time = pre_vel/3.6/pre_decel;
pre_decel_dist = 1/2*pre_decel*pre_decel_time^2;

pre_velmax_dist = pre_dist_target-pre_decel_dist-pre_accel_dist;
pre_velmax_time = pre_velmax_dist/(pre_vel/3.6);
pre_totaltime = pre_start+pre_accel_time+pre_velmax_time+pre_decel_time;

pre_decel_begin_dist = pre_dist_target-pre_decel_dist;

% Main cycle
% total_dist_target = 30; % m, target distance excluding pre-cycle
% vel_max = 15; % km/h
% accel = 0.1*9.81; % m/s^2
% decel = 0.15*9.81; % m/s^2
% accel_begin_time = 5;

main_cycle_delay = 5; % Time after the end of the pre-cycle the main cycle starts
end_delay = 4; % Time after the end of the main cycle the simulation ends

accel_time = vel_max/3.6/accel;
accel_dist = 1/2*accel*accel_time^2;

decel_time = vel_max/3.6/decel;
decel_dist = 1/2*decel*decel_time^2;

decel_begin_dist = total_dist_target;
vel_max_dist = total_dist_target - accel_dist;
vel_max_time = vel_max_dist/(vel_max/3.6);

cycle_total_time = pre_totaltime+main_cycle_delay+accel_time+vel_max_time+...
    decel_time+end_delay;

simu_max_time = cycle_total_time*4; % Maximum simulation time

% figure
% plot(cycle(:,1), cycle(:,2)*3.6, '-b'), grid on
% xlabel('Time (s)'), ylabel('Speed (km/h)')
% title('Driving cycle')

% cycle_dist = trapz(cycle(:,1), cycle(:,2));
cycle_dist = pre_dist_target + total_dist_target + decel_dist + 10;
ground_points = (0:ground_res:cycle_dist)';
ground_height = zeros(length(ground_points),1);
ground_dens = gamma_s*ones(length(ground_points),1);
ground_coh = c*ones(length(ground_points),1);
ground_kxy = k_xy*ones(length(ground_points),1);

%% Run sim

sim('eAgriBot_V4.slx')

%% Plot results

figure
subplot(4,1,1)
plot(SR_r.time, SR_r.data*100, '-b', SR_f.time, SR_f.data*100, '--r'), grid on
xlabel('Time (s)'), ylabel('Slip ratio (%)')
legend('Rear', 'Front')

subplot(4,1,2)
plot(T_Ft_r.time, T_Ft_r.data./r_rear/1e3, '-b', T_Ft_f.time, T_Ft_f.data/r_rear/1e3, '-r')
grid on
xlabel('Time (s)'), ylabel('Tire long. force (kN)')
legend('Rear', 'Front')

subplot(4,1,3)
plot(v_ms.time, v_ms.data*3.6, '-b', v_ms_cycle.time, v_ms_cycle.data*3.6, '-r')
grid on
xlabel('Time (s)'), ylabel('Speed (km/h)')
legend('Real', 'Cycle')

subplot(4,1,4)
plot(a_x.time, a_x.data, '-b'), grid on
xlabel('Time (s)'), ylabel('Acceleration (m/s^2)')

figure
plot(throttle.time, throttle.data ,'-b', brake.time, brake.data*(1-brake_bias), '-r',...
    throttle_f.time, throttle_f.data, '--m', brake_f.time, brake_f.data*brake_bias, '--k')
grid on
xlabel('Time (s)'), ylabel('Pedal (-)')
legend('Throttle (R)', 'Brake (R)', 'Throttle (F)', 'Brake (F)')
ylim([-1.2 1.2])
sgtitle('Driver inputs')

figure
subplot(2,1,1)
plot(Mot_torque.time, Mot_torque.data, '-b', Mot_torque_f.time, Mot_torque_f.data, '-r'), grid on
xlabel('Time (s)'), ylabel('Torque (Nm)')
legend('Rear', 'Front')

subplot(2,1,2)
plot(Mot_power.time, Mot_power.data/1e3, '-b', Mot_power_f.time, Mot_power_f.data/1e3), grid on
xlabel('Time (s)'), ylabel('Power (kW)')
legend('Rear', 'Front')
sgtitle('Motor torque and power')

figure
subplot(2,1,1)
plot(Fz_r.time, Fz_r.data/1e3, '-b', Fz_f.time, Fz_f.data/1e3, '-r')
grid on
xlabel('Time (s)'), ylabel('Vertical load (kN)')
legend('Rear axle', 'Front axle')

subplot(2,1,2)
plot(Fz_r.time, Fz_f.data./(Fz_f.data+Fz_r.data)*100, '-b')
grid on
xlabel('Time (s)'), ylabel('Front load fraction (%)')

figure
plot(w_tire_r.time, w_tire_r.data, '-b', w_tire_f.time,...
    w_tire_f.data, '-r')
grid on
xlabel('Time (s)'), ylabel('Wheel speed (rad/s)')
legend('Rear', 'Front')

figure
plot(Bat_energy_consumption.time, Bat_energy_consumption.data/3600, '-b',...
    Bat_internal_losses.time, Bat_internal_losses.data/3600, '-r')
grid on
legend('Total consumption', 'Battery losses', 'Location', 'Northwest')
xlabel('Time (s)'), ylabel('Energy (Wh)')

% subplot(1,2,2)
% plot(Bat_internal_losses.time, Bat_internal_losses.data/3600, '-r')
% xlabel('Time (s)'), ylabel('Battery losses (Wh)')

figure
plot(Bat_soc.time, Bat_soc.data, '-b'), grid on
xlabel('Time (s)'), ylabel('SOC (%)')
sgtitle('Battery state-of-charge')

figure
plot(T_Ft_r.time, T_Ft_r.data/r_rear, '-b', T_Ft_f.time, T_Ft_f.data/r_front, '-r')
grid on
xlabel('Time (s)'), ylabel('Force (N)')
legend('Rear', 'Front')
sgtitle('Long. tire forces on each axle')

figure
plot(RT_z.time, RT_z.data, '-b', RT_z_relative.time, RT_z_relative.data, '--k',...
    FT_z.time, FT_z.data, '-r'), grid on
xlabel('Time (s)'), ylabel('Position (m)')
legend('Rear', 'Rear rel.', 'Front')
sgtitle('Tire vertical position')

figure
subplot(2,2,1)
plot(Tines_Fx.time, Tines_Fx.data, '-b', Tines_Fz.time, Tines_Fz.data, '-r')
grid on
xlabel('Time (s)'), ylabel('Tine force (N)')
legend('X', 'Z')

subplot(2,2,2)
plot(B_tine_angle.time, rad2deg(B_tine_angle.data), '-b', F_tine_angle.time, rad2deg(F_tine_angle.data), '-r')
grid on
xlabel('Time (s)'), ylabel('Tine angle (deg)')
legend('Back row', 'Front row')

subplot(2,2,3)
plot(B_tine_spd.time, B_tine_spd.data*3.6, '-b', F_tine_spd.time, F_tine_spd.data*3.6, '-r')
grid on
xlabel('Time (s)'), ylabel('Tine speed (km/h)')

subplot(2,2,4)
plot(B_tine_depth.time, B_tine_depth.data, '-b', F_tine_depth.time, F_tine_depth.data, '-r',...
    Har_tire_z.time, Har_tire_z.data, '-k')
grid on
xlabel('Time (s)'), ylabel('Depth (m)')
legend('Tine back row', 'Tine front row', 'Harrow tire')

% figure
% plot(F_sigma_anglef.time, rad2deg(F_sigma_anglef.data), '-b',...
%     F_tau_anglef.time, rad2deg(F_tau_anglef.data), '-r',...
%     F_sigma_angler.time, rad2deg(F_sigma_angler.data), '-m',...
%     F_tau_angler.time, rad2deg(F_tau_angler.data), '-k')
% grid on
% xlabel('Time (s)')
% ylabel('Angle (deg)')
% legend('Sigma f', 'Tau f', 'Sigma r', 'Tau r')

% Find the instance of time when vel_max section began in the main cycle
main_cycle_begin = main_cycle_delay+precycle_endtime;
mc_ind = round(main_cycle_begin/step_size);
vcheck = 0;
loopvar = mc_ind;
while vcheck == 0
    if v_ms_cycle.data(loopvar) >= vel_max/3.6
        vcheck = 1;
    else
        loopvar = loopvar+1;
    end
end

ee = round(maincycle_dbegintime/step_size);
avg_spd_velmax = mean(v_ms.data(loopvar:ee)*3.6);
avg_dep_velmax = mean(F_tine_depth.data(loopvar:ee)*100);
avg_depb_velmax = mean(B_tine_depth.data(loopvar:ee)*100);

cons_mc = (Bat_energy_consumption.data(end)-Bat_energy_consumption.data(mc_ind-1))/3600;
cons_mc_whkm = cons_mc/trapz(v_ms.time(mc_ind:end), v_ms.data(mc_ind:end))*1000;

fprintf('The energy consumption during the main cycle was %.2f Wh or %.2f Wh/km.\n', cons_mc, cons_mc_whkm)
% fprintf('The overall energy consumption was %.2f Wh or %.2f Wh/km.\n', Bat_energy_consumption.data(end)/3600, Bat_energy_consumption.data(end)/3600/trapz(v_ms.time, v_ms.data)*1000)

fprintf('The main cycle began at %.3f s.\n', main_cycle_begin)
fprintf('The constant speed section began at %.3f s.\n', v_ms_cycle.time(loopvar))
fprintf('Deceleration (main cycle) began at %.3f s.\n', maincycle_dbegintime)

fprintf('The average speed during the constant speed section was %.2f km/h.\n', avg_spd_velmax)
fprintf('The average front tine depth during the constant speed section was %.2f cm.\n', avg_dep_velmax)
fprintf('The average back tine depth during the constant speed section was %.2f cm.\n', avg_depb_velmax)

avg_dep_total_velmax = (avg_dep_velmax+avg_depb_velmax)/2;

fprintf('The total average tine depth during the constant speed section was %.2f cm.\n', avg_dep_total_velmax)

%

figure
subplot(3,1,1)
plot(SR_har.time, SR_har.data*100, '-b'), grid on
xlabel('Time (s)'), ylabel('Tire slip ratio (%)')
legend('Harrow')

subplot(3,1,2)
plot(T_Ft_har.time, T_Ft_har.data./Har_tire_rad/1e3, '-b')
grid on
xlabel('Time (s)'), ylabel('Tire long. force (harrow) (kN)')

subplot(3,1,3)
plot(Har_tire_z.time, Har_tire_z.data*100, '-b')
grid on
xlabel('Time (s)'), ylabel('Harrow tire depth (cm)')

sgtitle('Harrow')
