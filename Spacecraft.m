% Project Spacecraft Control : Mathias BRACH and Lucie LINOTTE
clc
clear
close all
FigSet;
% Data and constants
max_speed = 50*2*pi/60; % in [rad/s]
beta = 63.4*pi/180; % in RADIANS
int_resist = 6; % in OHMS
torque_const = 1; %[Nm/A]%redefined later
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
[phi_roll,Iw_roll,H_roll,T_roll,t_roll,t_roll1,t_roll2,p] = maneuver (max_speed,roll_time_change,Ixx,beta,roll_change_angle/2,"Roll");
%% 1.2) Pitch
% The pitch motion is operated over 5s, meaning that the spacecraft has to
% go back to rest once the 5s are over. Two reaction wheels are
% needed to operate the roll maneuvre: reaction wheels 2 and 4. Yaw should
% be avoided during the roll maneuvre, thus Omega2=-Omega4. Since the
% rotation angle has to be positive, reaction wheel 1 has to turn
% clockwise, its angular velocity is thus negative. In this manner, Omega1
% will first go from 0 to -7000 RPM and then from -7000 RPM back to zero.
% Each phase lasts 2.5 s.
[phi_pitch,Iw_pitch,H_pitch,T_pitch,t_pitch,t_pitch1,t_pitch2,q] = maneuver (max_speed,pitch_time_change,Iyy,beta,pitch_change_angle/2,"Pitch");
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
[phi_yaw,Iw_yaw,H_yaw,T_yaw,t_yaw,t_yaw1,t_yaw2,z] = maneuver (max_speed,yaw_time,Izz,beta,phi_man_yaw/2,"Yaw");

%% 1.5) Wheel inertia and dimensions 
Iw = max([Iw_roll Iw_pitch Iw_yaw]);
Omega_max_roll=max(p)*Ixx/(2*Iw*sin(beta));
Omega_roll1=linspace(0,-Omega_max_roll,length(t_roll1));
Omega_roll2=linspace(0,Omega_max_roll,length(t_roll2));
Omega_roll=[Omega_roll1 Omega_roll2+Omega_roll1(end)];
Omega_pitch1=linspace(0,-max_speed,length(t_pitch1));
Omega_pitch2=linspace(0,max_speed,length(t_pitch2));
Omega_pitch=[Omega_pitch1 Omega_pitch2+Omega_pitch1(end)];
Omega_max_yaw=max(z)*Izz/(4*Iw*cos(beta));
t1=0:0.1:yaw_torque_time;
Omega_yaw1=linspace(0,Omega_max_yaw,length(t_yaw1));
Omega_yaw2=linspace(0,-Omega_max_yaw,length(t_yaw2));
Omega_yaw=[Omega_yaw1 Omega_yaw2+Omega_yaw1(end)];
Omega_yaw=[zeros(1,length(t1)) Omega_yaw];
% assuming a radius 3 times larger than the height of the wheel.
h=(2*Iw/(81*pi*steel_dens))^(1/5);
R=1.5*h;

fprintf('The diameter of the wheel is equal to %.2f cm and its height to %.2f cm \n',2*R*100,h*100);

%% 1.4) Plot profiles
figRoll=figure;
%sgtitle("Roll with I$_w$="+Iw+" and $\phi_{end}$="+phi_roll(end)*180/pi);
subplot(2,2,1);
plot(t_roll,phi_roll*180/pi,'color',[0 112/256 127/256]);
grid on
ylabel('$\phi$ [$^\circ$]');
xlabel('t [s]');
%title('Angle of rotation vs time');
subplot(2,2,2);
plot(t_roll,H_roll,'color',[0 112/256 127/256]);
grid on
%title('Angular momentum vs time');
ylabel('H [Nms]');
xlabel('t [s]');
subplot(2,2,3);
plot(t_roll,T_roll,'color',[0 112/256 127/256]);
grid on
%title('Torque vs time');
ylabel('T [Nm]');
xlabel('t [s]');
subplot(2,2,4);
plot(t_roll,Omega_roll*30/pi,'color',[0 112/256 127/256]);
grid on
%title('Rotation speed vs time');
ylabel('$\Omega$ [RPM]');
xlabel('t [s]');
%hgexport(figRoll,'roll_profile1.eps');

figPitch=figure;
%sgtitle("Pitch with I$_w$="+Iw+" and $\phi_{end}$="+phi_pitch(end)*180/pi);
subplot(2,2,1);
plot(t_pitch,phi_pitch*180/pi,'color',[0 112/256 127/256]);
grid on
ylabel('$\phi$ [$^\circ$]');
xlabel('t [s]');
xlim([0 pitch_time_change]);
%title('Angle of rotation vs time');
subplot(2,2,2);
plot(t_pitch,H_pitch,'color',[0 112/256 127/256]);
grid on
%title('Angular momentum vs time');
ylabel('H [Nms]');
xlabel('t [s]');
xlim([0 pitch_time_change]);
subplot(2,2,3);
plot(t_pitch,T_pitch,'color',[0 112/256 127/256]);
grid on
%title('Torque vs time');
ylabel('T [Nm]');
xlabel('t [s]');
xlim([0 pitch_time_change]);
subplot(2,2,4);
plot(t_pitch,Omega_pitch*30/pi,'color',[0 112/256 127/256]);
grid on
%title('Rotation speed vs time');
ylabel('$\Omega$ [RPM]');
xlabel('t [s]');
xlim([0 pitch_time_change]);
%hgexport(figPitch,'pitch_profile1.eps');

figYaw=figure;
%sgtitle("Yaw with I$_w$="+Iw+" and $\phi_{end}$="+phi_yaw(end)*180/pi);
subplot(2,2,1);
plot(t_yaw,phi_yaw*180/pi,'color',[0 112/256 127/256]);
grid on
ylabel('$\phi$ [$^\circ$]');
xlabel('t [s]');
xlim([0 5.5]);
%title('Angle of rotation vs time');
subplot(2,2,2);
plot(t_yaw,H_yaw,'color',[0 112/256 127/256]);
grid on
%title('Angular momentum vs time');
ylabel('H [Nms]');
xlabel('t [s]');
xlim([0 5.5]);
subplot(2,2,3);
plot(t_yaw,T_yaw,'color',[0 112/256 127/256]);
grid on
%title('Torque vs time');
ylabel('T [Nm]');
xlabel('t [s]');
xlim([0 5.5]);
subplot(2,2,4);
plot(t_yaw,Omega_yaw*30/pi,'color',[0 112/256 127/256]);
grid on
%title('Rotation speed vs time');
ylabel('$\Omega$ [RPM]');
xlabel('t [s]');
xlim([0 5.5]);
%hgexport(figYaw,'yaw_profile1.eps');

%% 1.6) Compute current and voltage needed
N=60; % justifier valeur.
% Roll. Assumptions: i1=-i3, e1=-e3, omega1=-omega3 and omega1,max=-7000RPM
pdot=T_roll/Ixx;
i_roll=1/N*(Ixx*pdot/(2*sin(beta))+c*(p*sin(beta)-Omega_roll));
e_roll=2*(int_resist*abs(i_roll)+N*p*sin(beta)-Omega_roll*N);
figure
plot(t_roll,i_roll,'color',[0 112/256 127/256]);
%title('Roll: Current vs time');
xlabel('t [s]');
ylabel('i [A]');
%hgexport(gcf,'i_roll.eps');
% ylim([min(i_roll)  max(i_roll)]);
figure
plot(t_roll,e_roll);
title('Roll: Voltage vs time');
xlabel('t [s]');
ylabel('e [V]');

% Pitch. Same assumptions as roll 
qdot=T_pitch/Iyy;
i_pitch=1/N*(Iyy*qdot/(2*sin(beta))+c*(q*sin(beta)-Omega_pitch));
e_pitch=2*(int_resist*abs(i_pitch)+N*q*sin(beta)-Omega_pitch*N);
figure
plot(t_pitch,i_pitch,'color',[0 112/256 127/256]);
%title('Pitch: Current vs time');
xlabel('t [s]');
ylabel('i [A]');
%hgexport(gcf,'i_pitch.eps');
% ylim([min(i_pitch)  max(i_pitch)]);
figure
plot(t_pitch,e_pitch);
title('Pitch: Voltage vs time');
xlabel('t [s]');
ylabel('e [V]');

% Yaw

zdot=T_yaw/Izz;
zdot(1:length(t1))=0;
i_yaw=1/N*(Izz*zdot/(4*cos(beta))+c*(z*cos(beta)-Omega_yaw));
e_yaw=4*(int_resist*abs(i_yaw)+N*z*cos(beta)-Omega_yaw*N);
figure
plot(t_yaw,i_yaw,'color',[0 112/256 127/256]);
%title('Yaw: Current vs time');
xlabel('t [s]');
ylabel('i [A]');
%hgexport(gcf,'i_yaw.eps');
% ylim([min(i_yaw)  max(i_yaw)]);
figure
plot(t_yaw,e_yaw);
title('Yaw: Voltage vs time');
xlabel('t [s]');
ylabel('e [V]');
%% 2) State space representation

%2.1) Roll
Ar=[0,1;0,sin(beta)/Ixx*(N^2/int_resist+c)*(-2*sin(beta)-Ixx/(Iw*sin(beta)))];
Br=[0;sin(beta)*N/(Ixx*int_resist)];
Cr=[1,0;0,1];
Dr=[0;0];
%2.2) Pitch
Ap=[0,1;0,sin(beta)/Iyy*(N^2/int_resist+c)*(-2*sin(beta)-Iyy/(Iw*sin(beta)))];
Bp=[0;sin(beta)*N/(Iyy*int_resist)];
Cp=[1,0;0,1];
Dp=[0;0];
%2.3) Yaw
Hzz=H_yaw(6);
Ay=[0,1;0,cos(beta)/Izz*(N^2/int_resist+c)*(-2*cos(beta)+(Hzz-Izz)/(Iw*cos(beta)))];
By=[0;cos(beta)*N/(Izz*int_resist)];
Cy=[1,0;0,1];
Dy=[0;0];
%% 3)Roll LQR
Qr = [1e5 0; 0 1];
Rr = 0.23;
Kr = lqr(Ar,Br,Qr,Rr);
CLr = Ar - Br*Kr;
% Ccr = [Cr];
% Ddr = [Dr];
RLr = eig(CLr);
sysr_lqr = ss(CLr,Br,Cr,Dr);
x0 = [-pi/2 0].';
t = linspace(0,5,1000);
% [T, eigR] = eig(Ar);
% xt = zeros(length(t),2);
% for i = 1:length(t)
%     xt(i,:) = T*exp(eigR*t(i))*T^(-1) * x0;
% end

% figure
% plot(t,xt(:,1))
[Tk, Dk] = eig(Ar - Br*Kr);
xtk = zeros(length(t),2);
for i = 1:length(t)
    xtk(i,:) = Tk*expm(Dk*t(i))*Tk^(-1) * x0 + pi/2;
end
figure
plot(t,xtk(:,1))
title('Roll angle LQR');
figure
plot(t,xtk(:,2))
title('Roll angle rate LQR');

[numr, denr] = ss2tf(Ar, Br, Cr, Dr);
H_phir = tf(numr(1,:), denr);
L_phir = Kr(1) * H_phir;
figure
margin(L_phir);

figure
nyquist(L_phir);
%% LQR Pitch
Qp = [1e5 0; 0 1];
Rp = 0.02;
Kp = lqr(Ap,Bp,Qp,Rp);
CLp = Ap - Bp*Kp;
Ccp = [Cp; -Kp];
Ddp = [Dp; 0];
RLp = eig(CLp);
sysr_lqp = ss(CLp,Bp,Ccp,Ddp);
x0_p = [phi_pitch(end) 0].';
[Tp, eigP] = eig(Ap);
xt_p = zeros(length(t),2);
for i = 1:length(t)
    xt_p(i,:) = Tp*expm(eigP*t(i))*Tp^(-1) * x0_p ;
end

% figure
% plot(t,xt_p(:,1))
[Tk_p, Dk_p] = eig(Ap - Bp*Kp);
xtk_p = zeros(length(t),2);
for i = 1:length(t)
    xtk_p(i,:) = Tk_p*expm(Dk_p*t(i))*Tk_p^(-1) * x0_p;
end
figure
plot(t,xtk_p(:,1))
title('Pitch angle LQR');
figure
plot(t,xtk_p(:,2))
title('Pitch angle rate LQR')

[nump, denp] = ss2tf(Ap, Bp, Cp, Dp);
H_phip = tf(nump(1,:), denp);
L_phip = Kp(1) * H_phip;
figure
margin(L_phip);

figure
nyquist(L_phip);
% close all

% Requirements

%% LQR Yaw
Qy = [1e5 0; 0 1];
Ry = .025;
Ky = lqr(Ay,By,Qy,Ry);
CLy = Ay - By*Ky;
Ccy = [Cy; -Ky];
Ddy = [Dy; 0];  
RLy = eig(CLy);
sysr_lqy = ss(CLy,By,Ccy,Ddy);
x0_y = [max(phi_yaw) 0].';
[Ty, eigY] = eig(Ay);
xt_y = zeros(length(t),2);
for i = 1:length(t)
    xt_y(i,:) = Ty*expm(eigY*t(i))*Ty^(-1) * x0_y;
end

% figure
% plot(t,xt_y(:,1))
[Tk_y, Dk_y] = eig(Ay - By*Ky);
xtk_y = zeros(length(t),2);
for i = 1:length(t)
    xtk_y(i,:) = Tk_y*expm(Dk_y*t(i))*Tk_y^(-1) * x0_y;
end
figure
plot(t,xtk_y(:,1))
title('Yaw angle LQR');
figure
plot(t,xtk_y(:,2))
title('Yaw angle rate LQR');

[numy, deny] = ss2tf(Ay, By, Cy, Dy);
H_phiy = tf(numy(1,:), deny);
L_phiy = Ky(1) * H_phiy;
figure
margin(L_phiy);

figure
nyquist(L_phiy);

%% 4) PID
%Roll
%[br,ar]=ss2tf(Co*Ar,Co*Br,Cr,Dr);
s=tf('s');
Hr_pid=(N*sin(beta)/(int_resist*Ixx))/(s^2+s*(sin(beta)/Ixx*(N^2/int_resist+c)*(2*sin(beta)+Ixx/(Iw*sin(beta)))));
[Arm,Brm,Crm,Drm]=tf2ss([0 0 N*sin(beta)/(int_resist*Ixx);0 N*sin(beta)/(int_resist*Ixx) 0],[1 -(sin(beta)/Ixx*(N^2/int_resist+c)*(2*sin(beta)+Ixx/(Iw*sin(beta)))) 0]);

%sisotool(Hr_pid)
%Pitch
[bp,ap]=ss2tf(Ap,Bp,Cp,Dp);
Hp=tf(bp(2,:),ap); %get transfer function related to roll rate and voltage difference.
s=tf('s');
Hp_pid=N*sin(beta)/(int_resist*Iyy)/(s^2+s*(sin(beta)/Iyy*(N^2/int_resist+c)*(2*sin(beta)+Iyy/(Iw*sin(beta)))));
%sisotool(Hp_pid)
%Yaw
[by,ay]=ss2tf(Ay,By,Cy,Dy);
Hy=tf(by(2,:),ay); %get transfer function related to roll rate and voltage difference.
Hy_pid=N*cos(beta)/(int_resist*Izz)/(s^2+s*(cos(beta)/Izz*(N^2/int_resist+c)*(2*cos(beta)+(Izz-Hzz)/(Iw*cos(beta)))));
%sisotool(Hy_pid)
z0=[phi_man_yaw z(7)];
[Aym,Bym,Cym,Dym]=tf2ss([0 0 N*cos(beta)/(int_resist*Izz);0 N*cos(beta)/(int_resist*Izz) 0],[1 -(cos(beta)/Izz*(N^2/int_resist+c)*(2*cos(beta)+(Izz-Hzz)/(Iw*cos(beta)))) 0]);
%%


function [Atot] = matArr(pos1,pos2,mat,Atot)
if size(Atot,2) > 1
    Atot(pos1,pos1) = mat(1,1);
    Atot(pos2,pos1) = mat(2,1);
    Atot(pos1,pos2) = mat(1,2);
    Atot(pos2,pos2) = mat(2,2);
else
    Atot(pos1) = mat(1);
    Atot(pos2) = mat(2);
end
end
