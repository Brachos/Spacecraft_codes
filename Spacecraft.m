% Project Spacecraft Control : Mathias BRACH and Lucie LINOTTE
clc
clear all
close all

pt=11;
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

% w = H/10; % width, in METERS
% R = 2*w; % radius, in METERS
%M = steel_dens * pi * R^2 * w; %[kg]
%Iw = 1/2 * M * R^2; %inertia of the wheel, in [kg m^2]

%roll
p = roll_change_angle/roll_time_change; %[rad/s]
p_dot=2*roll_change_angle/roll_time_change^2;

%pitch
q = pitch_change_angle/pitch_time_change; %[rad/s]
q_dot=2*pitch_change_angle/pitch_time_change^2;

%yaw, phase 1
t_yaw=0:0.1:yaw_torque_time+yaw_time;
index=find(t_yaw==yaw_torque_time);
phi_yaw=zeros(1,length(t_yaw));
H_yaw=zeros(1,length(t_yaw));
T_yaw=zeros(1,length(t_yaw));
r_dot1=yaw_torque/Izz;
yaw_angle_change1=r_dot1*yaw_torque_time/2;
r1 = yaw_angle_change1/yaw_torque_time; %[rad/s]

for i=1:index
     phi_yaw(i)=1/2*r_dot1*t_yaw(i)^2;
     H_yaw(i)=Izz*r_dot1*t_yaw(i);
     T_yaw(i)=yaw_torque; 
end
%yaw, phase 2
r2=-phi_yaw(index)/yaw_time; %[rad/s]
r_dot2=-2*phi_yaw(index)/(yaw_time^2);
C1=r_dot1*yaw_torque_time-1/2*r_dot2;
C2=phi_yaw(index)-1/8*r_dot2-1/2*C1;
for i=index+1:length(t_yaw)
     phi_yaw(i)=1/2*r_dot2*t_yaw(i)^2+C1*t_yaw(i)+C2;
     H_yaw(i)=Izz*r_dot2*t_yaw(i);
     T_yaw(i)=Izz*r_dot2;
end
r=max([r1;r2]);

omega = [p; q; r];

%H_min = [Ixx*p; Iyy*q; Izz*r] + A*[Iw*max_speed; Iw*max_speed; Iw*max_speed; Iw*max_speed];

%roll
Iw_roll=p*Ixx/(2*max_speed*sin(beta));
%pitch
Iw_pitch=q*Iyy/(2*max_speed*sin(beta));
%yaw
Iw_yaw=r*Izz/(4*max_speed*cos(beta));

Iw=max([Iw_roll;Iw_pitch;Iw_yaw]);
%deduce dimension of the wheels
h=(2*Iw/(81*pi*steel_dens))^(1/5);
R=3*h;
%%
% roll
t_roll=0:0.1:roll_time_change;
phi_roll=1/2*p_dot*t_roll.^2;
H_roll= Ixx*p_dot*t_roll;
figure
sgtitle('Roll');
subplot(3,1,1);
plot(t_roll,phi_roll*180/pi);
ylabel('$\phi$[�]','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);
title('Angle of rotation vs time');
subplot(3,1,2);
plot(t_roll,H_roll);
title('Angular momentum vs time');
ylabel('H [Nms]','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);
subplot(3,1,3);
T_roll=Ixx*p_dot*ones(1,length(t_roll));
plot(t_roll,T_roll);
title('Torque vs time');
ylabel('T [Nm]','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);

% pitch
t_pitch=0:0.1:pitch_time_change;
phi_pitch=1/2*q_dot*t_pitch.^2;
H_pitch= Iyy*q_dot*t_pitch;
figure
sgtitle('Pitch');
subplot(3,1,1);
plot(t_pitch,phi_pitch*180/pi);
ylabel('$\phi$[�]','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);
title('Angle of rotation vs time');
subplot(3,1,2);
plot(t_pitch,H_pitch);
title('Angular momentum vs time');
ylabel('H [Nms]','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);
subplot(3,1,3);
T_pitch=Iyy*q_dot*ones(1,length(t_pitch));
plot(t_pitch,T_pitch);
title('Torque vs time');
ylabel('T [Nm]','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);

% yaw
figure
sgtitle('Yaw');
subplot(3,1,1);
plot(t_yaw,phi_yaw*180/pi);
ylabel('$\phi$[�]','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);
title('Angle of rotation vs time');
subplot(3,1,2);
plot(t_yaw,H_yaw);
title('Angular momentum vs time');
ylabel('H [Nms]','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);
subplot(3,1,3);
plot(t_yaw,T_yaw);
title('Torque vs time');
ylabel('T [Nm]','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);