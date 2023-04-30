% Project Spacecraft Control : Mathias BRACH and Lucie LINOTTE
clc
close all

% Data and constants
max_speed = 7000*2*pi/60; % in [rad/s]
beta = 63.4*pi/180; % in RADIANS
int_resist = 50; % in OHMS
torque_const = 1; %[Nm/A]
damp_fact = 1e-4; %[Nm/(rad/s)]
steel_dens = 8000; %[kg/m^3]

% Inertia
Ixx = 6000; %[kg m^2]
Iyy = 50000; %[kg m^2]
Izz = 50000; %[kg m^2]

% Dimensions
L = 13.4; %[m]
W = 11.76; %[m]
H = 2.4; %[m]

% Requierements
roll_change_angle = 90*pi/180; %in RADIANS
roll_time_change = 3; %[s]
roll_time_stab = 10; %[s]
roll_overshoot = 0.2; %[-]
roll_accu = 0.02; %[-]
pitch_change_angle = 30*pi/180; %in RADIANS
pitch_time_change = 5; %[s]
pitch_overshoot = 0.05; %[-]
yaw_torque = 4000; %[Nm]
yaw_torque_time = 0.5; %[s]
yaw_time = 5; %[s]
yaw_accu = 0.05; %[-]

%% Steps from the Hints
% 1) Minimum torque and momentum 
A = [sin(beta) 0 -sin(beta) 0;
    0 sin(beta) 0 -sin(beta);
    cos(beta) cos(beta) cos(beta) cos(beta)];

w = H/10; % width, in METERS
R = 2*w; % radius, in METERS
M = steel_dens * pi * R^2 * w; %[kg]
Iw = 1/2 * M * R^2; %inertia of the wheel, in [kg m^2]
p = roll_change_angle/roll_time_change; %[rad/s]
q = pitch_change_angle/pitch_time_change; %[rad/s]
r = (yaw_torque/Izz * yaw_torque_time^2/2)/yaw_time; %[rad/s]
omega = [p; q; r];

H_min = [Ixx*p; Iyy*q; Izz*r] + A*[Iw*max_speed; Iw*max_speed; Iw*max_speed; Iw*max_speed];
