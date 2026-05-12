clc;
clear all;
close all;

%% Constants
g = 9.81;              % Gravity [m/s^2]
rho = 1.225;           % Air density [kg/m^3]

%% Rider & Bike Parameters
m_rider = 75;          % Rider mass [kg]
m_bike = 10;           % Bike mass [kg]
m = m_rider + m_bike;
CdA = 0.3;             % Drag coefficient * area
Crr = 0.005;           % Rolling resistance
power_target = 250;    % Rider constant power [W]
wind_speed = 2;        % Headwind [m/s] (positive is headwind)

%% Route (distance vs elevation)
route = linspace(0, 5000, 500);     % 5 km route
elevation = 20*sin(route/800);      % synthetic hills
dx = route(2) - route(1);           % distance step
slope = gradient(elevation, dx);

%% Simulation Setup
v = zeros(size(route));
v(1) = 5;                           % initial speed [m/s]
time_total = 0;
power_inst = zeros(size(route));

%% Simulation Loop
for i = 2:length(route)
    % 1. Calculate resistances based on current speed
    theta = atan(slope(i));
    F_drag = 0.5 * rho * CdA * (v(i-1) + wind_speed)^2;
    F_roll = Crr * m * g * cos(theta);
    F_climb = m * g * sin(theta);
    F_resist = F_drag + F_roll + F_climb;
    
    % 2. Calculate Force from rider (Power = F * v)
    F_rider = power_target / max(v(i-1), 0.5); % max to avoid division by zero
    
    % 3. Calculate Acceleration (F_net = m * a)
    F_net = F_rider - F_resist;
    a = F_net / m;
    
    % 4. Update velocity (using kinematics: v_final^2 = v_initial^2 + 2*a*dx)
    v_sq = v(i-1)^2 + 2 * a * dx;
    v(i) = sqrt(max(v_sq, 0.1)); % ensure speed doesn't go negative or zero
    
    % 5. Update time
    dt = dx / v(i);
    time_total = time_total + dt;
    
    % 6. Record actual power used
    power_inst(i) = F_rider * v(i);
end

%% Results
fprintf('Total time: %.2f minutes\n', time_total/60);
fprintf('Average speed: %.2f km/h\n', mean(v)*3.6);

%% Plots
figure('Color', 'w')

subplot(3,1,1)
plot(route, elevation, 'g', 'LineWidth', 1.5)
ylabel('Elevation [m]')
title('Route Profile')
grid on

subplot(3,1,2)
plot(route, v * 3.6, 'b', 'LineWidth', 1.5)
ylabel('Speed [km/h]')
grid on

subplot(3,1,3)
plot(route, power_inst, 'r', 'LineWidth', 1.5)
xlabel('Distance [m]')
ylabel('Power [W]')
grid on