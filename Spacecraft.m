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
c = 1e-4; %[Nm/(rad/s)]
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
[phi_roll,Iw_roll,H_roll,T_roll,t_roll,p] = maneuver (max_speed,roll_time_change,Ixx,beta,roll_change_angle/2,"Roll");
%% 1.2) Pitch
% The pitch motion is operated over 5s, meaning that the spacecraft has to
% go back to rest once the 5s are over. Two reaction wheels are
% needed to operate the roll maneuvre: reaction wheels 2 and 4. Yaw should
% be avoided during the roll maneuvre, thus Omega2=-Omega4. Since the
% rotation angle has to be positive, reaction wheel 1 has to turn
% clockwise, its angular velocity is thus negative. In this manner, Omega1
% will first go from 0 to -7000 RPM and then from -7000 RPM back to zero.
% Each phase lasts 2.5 s.
[phi_pitch,Iw_pitch,H_pitch,T_pitch,t_pitch,q] = maneuver (max_speed,pitch_time_change,Iyy,beta,pitch_change_angle/2,"Pitch");
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
%Phase 1, the objective is to find the velocity and position of the
%spacecraft after 0.5s.
phi_man_yaw=1/2*yaw_torque/Izz*yaw_torque_time^2;
[phi_yaw,Iw_yaw,H_yaw,T_yaw,t_yaw,z] = maneuver (max_speed,yaw_time,Izz,beta,phi_man_yaw/2,"Yaw");


%% 1.4) Plot profiles
figure
<<<<<<< HEAD
sgtitle("Roll with I$_w$="+Iw_roll+" and $\phi_{end}$="+phi_roll(end)*180/pi,'interpreter','latex','Fontsize',pt);
=======
sgtitle("Roll with I$_w$="+Iw+" and $\phi_{end}$="+phi_roll(end)*180/pi);
>>>>>>> main
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
<<<<<<< HEAD
sgtitle("Pitch with I$_w$="+Iw_pitch+" and $\phi_{end}$="+phi_pitch(end)*180/pi,'interpreter','latex','Fontsize',pt);
=======
sgtitle("Pitch with I$_w$="+Iw+" and $\phi_{end}$="+phi_pitch(end)*180/pi);
>>>>>>> main
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
<<<<<<< HEAD
sgtitle("Yaw with I$_w$="+Iw_yaw+" and $\phi_{end}$="+phi_yaw(end)*180/pi,'interpreter','latex','Fontsize',pt);
subplot(3,1,1);
plot(t_yaw,phi_yaw*180/pi);
ylabel('$\phi$','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);
xlim([0 5.5]);
=======
sgtitle("Yaw with I$_w$="+Iw+" and $\phi_{end}$="+phi_yaw(end)*180/pi);
subplot(3,1,1);
plot(t_yaw,phi_yaw*180/pi);
ylabel('$\phi$');
xlabel('t [s]');
>>>>>>> main
title('Angle of rotation vs time');
subplot(3,1,2);
plot(t_yaw,H_yaw);
title('Angular momentum vs time');
<<<<<<< HEAD
ylabel('H [Nms]','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);
xlim([0 5.5]);
subplot(3,1,3);
plot(t_yaw,T_yaw);
title('Torque vs time');
ylabel('T [Nm]','interpreter','latex','Fontsize',pt);
xlabel('t [s]','interpreter','latex','Fontsize',pt);
xlim([0 5.5]);
%% 1.5) Wheel inertia and dimensions 
Iw = max([Iw_roll Iw_pitch Iw_yaw]);
% [phi_roll2,Iw,H_roll2,T_roll2,t_roll2] = maneuver (max_speed,roll_time_change,Ixx,roll_change_angle/2,beta,"Roll",Iw);
% %[Iw,phi_roll,H_roll,T_roll,t_roll] = maneuver (max_speed,roll_time_change,Ixx,pi/2,beta,"Roll",Iw);
% [phi_pitch2,Iw,H_pitch2,T_pitch2,t_pitch2] = maneuver (max_speed,pitch_time_change,Iyy,pitch_change_angle/2,beta,"Pitch",Iw);
% [phi_yaw2,Iw,H_yaw2,T_yaw2,t_yaw2] = maneuver (max_speed,yaw_time,Izz,beta,phi_man_yaw/2,"Yaw",Iw);
% assuming a radius 3 times larger than the height of the wheel.
h=(2*Iw/(81*pi*steel_dens))^(1/5);
R=3*h;

fprintf('The diameter of the wheel is equal to %.2f cm and its height to %.2f cm \n',R*100,h*100);
%% 1.6) Compute current and voltage needed
N=30; % ???????? justifier valeur.
% Roll. Assumptions: i1=-i3, e1=-e3, omega1=-omega3 and omega1,max=-7000RPM
pdot=2*roll_change_angle/roll_time_change^2;
i_roll=Ixx*pdot/(2*N*sin(beta));
pmax=p(find(t_roll==roll_time_change/2,1,'first'));
e_roll=int_resist*i_roll+N*pmax*sin(beta)+N*max_speed;
% Pitch. Same assumptions as roll 
qdot=2*pitch_change_angle/pitch_time_change^2;
i_pitch=Iyy*qdot/(2*N*sin(beta));
qmax=q(find(t_pitch==pitch_time_change/2,1,'first'));
e_pitch=int_resist*i_pitch+N*qmax*sin(beta)+N*max_speed;
% Yaw
zdot=(yaw_torque/Izz+2*phi_man_yaw/yaw_time^2);
i_yaw=Izz*zdot/(4*N*cos(beta));
zmax=z(find(t_yaw==yaw_time/2,1,'first'));
e_yaw=int_resist*i_yaw+N*zmax*cos(beta)+2*N*max_speed;
%% 2) State space representation

%2.1) Roll
Ar=[0,1;0,sin(beta)/Ixx*(N^2/R+c)*(-2*sin(beta)-Ixx/(Iw*sin(beta)))];
Br=[0;sin(beta)*N/(Ixx*R)];
Cr=[1,0;0,1];
Dr=0;
%2.2) Pitch
Ap=[0,1;0,sin(beta)/Iyy*(N^2/R+c)*(-2*sin(beta)-Iyy/(Iw*sin(beta)))];
Bp=[0;sin(beta)*N/(Iyy*R)];
Cp=[1,0;0,1];
Dp=0;
%2.3) Yaw
Ay=[0,1;0,cos(beta)/Izz*(N^2/R+c)*(-2*cos(beta)-Izz/(Iw*cos(beta)))];
By=[0;cos(beta)*N/(Izz*R)];
Cy=[1,0;0,1];
Dy=0;
%% 3)LQR
%% 4) PDI
=======
ylabel('H [Nms]');
xlabel('t [s]');
subplot(3,1,3);
plot(t_yaw,T_yaw);
title('Torque vs time');
ylabel('T [Nm]');
xlabel('t [s]');
>>>>>>> main
