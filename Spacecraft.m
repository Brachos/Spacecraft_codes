% Project Spacecraft Control : Mathias BRACH and Lucie LINOTTE
clc
clear
close all
FigSet;
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


%% 1.1) Roll
% The roll motion is operated over 3s, meaning that the spacecraft has to
% go back to rest once the 3s are over. Two reaction wheels are
% needed to operate the roll maneuvre: reaction wheels 1 and 3. Yaw should
% be avoided during the roll maneuvre, thus Omega1=-Omega3. Since the
% rotation angle has to be positive, reaction wheel 1 has to turn
% clockwise, its angular velocity is thus negative. In this manner, Omega1
% will first go from 0 to -7000 RPM and then from -7000 RPM back to zero.
% Each phase lasts 1.5 s.
[Iw_roll,phi_roll,H_roll,T_roll,t_roll] = maneuver (max_speed,roll_time_change,Ixx,pi/2,beta,"Roll");
%% 1.2) Pitch
% The pitch motion is operated over 5s, meaning that the spacecraft has to
% go back to rest once the 5s are over. Two reaction wheels are
% needed to operate the roll maneuvre: reaction wheels 2 and 4. Yaw should
% be avoided during the roll maneuvre, thus Omega2=-Omega4. Since the
% rotation angle has to be positive, reaction wheel 1 has to turn
% clockwise, its angular velocity is thus negative. In this manner, Omega1
% will first go from 0 to -7000 RPM and then from -7000 RPM back to zero.
% Each phase lasts 2.5 s.
[Iw_pitch,phi_pitch,H_pitch,T_pitch,t_pitch] = maneuver (max_speed,pitch_time_change,Iyy,30*pi/180,beta,"Pitch");

%% 1.3) Yaw
% The yaw motion can be seperated into three phases. First, a constant
% torque is applied to the spacecraft during 0.5s. It will cause the
% spacecraft to rotate by a certain angle. At the end of the 0.5s, the
% spacecraft will have a certain velocity and will have to go back to rest
% in 5s. It will also need to retrieve its initial position by rotating of
% the same angle as in the first phase but negative this time. Its angular
% velocity will thus be positive. We have the following relation:
% Omega1=Omega2=Omega3=Omega4. The last two phases are similar to the ones
% for pitch and roll.
[Iw_yaw,phi_yaw,H_yaw,T_yaw,t_yaw] = maneuver (max_speed,yaw_time,Izz,0,beta,"Yaw");

%% 1.4) Wheel inertia and dimensions 
Iw = max([Iw_roll Iw_pitch Iw_yaw]);
[Iw,phi_roll,H_roll,T_roll,t_roll] = maneuver (4200*pi/30,roll_time_change,Ixx,pi/2,beta,"Roll",Iw);
%[Iw,phi_roll,H_roll,T_roll,t_roll] = maneuver (max_speed,roll_time_change,Ixx,pi/2,beta,"Roll",Iw);
[Iw,phi_pitch,H_pitch,T_pitch,t_pitch] = maneuver (max_speed,pitch_time_change,Iyy,30*pi/180,beta,"Pitch",Iw);
[Iw,phi_yaw,H_yaw,T_yaw,t_yaw] = maneuver (800*pi/30,yaw_time,Izz,0,beta,"Yaw",Iw);
% assuming a radius 3 times larger than the height of the wheel.
h=(2*Iw/(81*pi*steel_dens))^(1/5);
R=3*h;

fprintf('The diameter of the wheel is equal to %.2f cm and its heigh to %.2f cm \n',R*100,h*100);
%% 1.5) Plot profiles
figure
sgtitle("Roll with I$_w$="+Iw+" and $\phi_{end}$="+phi_roll(end)*180/pi);
subplot(3,1,1);
plot(t_roll,phi_roll*180/pi);
ylabel('$\phi$');
xlabel('t [s]');
title('Angle of rotation vs time');
subplot(3,1,2);
plot(t_roll,H_roll);
title('Angular momentum vs time');
ylabel('H [Nms]');
xlabel('t [s]');
subplot(3,1,3);
plot(t_roll,T_roll);
title('Torque vs time');
ylabel('T [Nm]');
xlabel('t [s]');


figure
sgtitle("Pitch with I$_w$="+Iw+" and $\phi_{end}$="+phi_pitch(end)*180/pi);
subplot(3,1,1);
plot(t_pitch,phi_pitch*180/pi);
ylabel('$\phi$');
xlabel('t [s]');
title('Angle of rotation vs time');
subplot(3,1,2);
plot(t_pitch,H_pitch);
title('Angular momentum vs time');
ylabel('H [Nms]');
xlabel('t [s]');
subplot(3,1,3);
plot(t_pitch,T_pitch);
title('Torque vs time');
ylabel('T [Nm]');
xlabel('t [s]');

figure
sgtitle("Yaw with I$_w$="+Iw+" and $\phi_{end}$="+phi_yaw(end)*180/pi);
subplot(3,1,1);
plot(t_yaw,phi_yaw*180/pi);
ylabel('$\phi$');
xlabel('t [s]');
title('Angle of rotation vs time');
subplot(3,1,2);
plot(t_yaw,H_yaw);
title('Angular momentum vs time');
ylabel('H [Nms]');
xlabel('t [s]');
subplot(3,1,3);
plot(t_yaw,T_yaw);
title('Torque vs time');
ylabel('T [Nm]');
xlabel('t [s]');