function [Iw,phi,H,T,t] = maneuver (Omega_max,t_maneuver,Is,man_angle,beta,mode,Iw)

Iw_vec=3:0.1:10;
phi_final=0.1;
n=1;
    
if mode=='Roll' || mode=='Pitch'
    while phi_final<man_angle
        if nargin== 6 
            Iw=Iw_vec(n);
        end
        % Phase 1
        t1=0:0.1:t_maneuver/2;
        omega1=linspace(0,Omega_max,length(t1));
        q1=2*omega1*Iw*sin(beta)/Is;
        qdot1=q1(end)/t1(end);
        phi1=1/2*qdot1*t1.^2;
        T1=Is*qdot1*ones(1,length(t1));

        % Phase 2
        t2=0:0.1:t_maneuver/2;
        omega2=linspace(0,-Omega_max,length(t2));
        q2=2*omega2*Iw*sin(beta)/Is;
        qdot2=q2(end)/t2(end);
        phi2=1/2*qdot2*t2.^2+q1(end).*t2+phi1(end);
        T2=Is*qdot2*ones(1,length(t2)); 

        % Total
        t=[t1 t2+t_maneuver/2];
        phi=[phi1 phi2];
        phi_final=phi(end);
        q=[q1 q2+q1(end)];
        H= Is*q; %angular momentum of the spacecraft.
        T= [T1 T2];
        
        n=n+1;
    end 
elseif mode=='Yaw'
    while phi_final>man_angle
        if nargin== 6 
            Iw=Iw_vec(n);
        end
        yaw_torque = 4000; %[Nm]
        yaw_torque_time = 0.5; %[s]
        
        %Phase 1, the objective is to find the velocity and position of the
        %spacecraft after 0.5s.
        zdot1=yaw_torque/Is;
        t1=0:0.1:yaw_torque_time;
        z1=zdot1.*t1; 
        phi1=1/2*zdot1*t1.^2; % rotation angle of the spacecraft over 0.5s
        T1=yaw_torque*ones(1,length(t1));
        omega_max1=-z1(end)*Is/(Iw*cos(beta)*4); % angular velocity of the spacecraft by 0.5s
        
        %Phase 2 
        t2=0:0.1:t_maneuver/2;
        omega2=linspace(omega_max1,Omega_max,length(t2));
        z2=-4*omega2*Iw*sin(beta)/Is;
        zdot2=z2(end)/t2(end);
        phi2=1/2*zdot2*t2.^2+z1(end).*t2+phi1(end);
        T2=Is*zdot2*ones(1,length(t2));
        
        %Phase 3
        t3=0:0.1:t_maneuver/2;
        omega3=linspace(0,-Omega_max,length(t3));
        z3=-4*omega3*Iw*sin(beta)/Is;
        zdot3=z3(end)/t3(end);
        phi3=1/2*zdot3*t3.^2+z2(end).*t2+phi2(end);
        T3=Is*zdot3*ones(1,length(t3));
        
        % Total
        t=[t1 t2+t1(end) t3+t_maneuver/2+t1(end)];
        phi=[phi1 phi2 phi3];
        phi_final=phi(end);
        z=[z1 z2+z1(end) z3+z2(end)+z1(end)];
        H= Is*z; %angular momentum of the spacecraft.
        T= [T1 T2 T3];
        
        n=n+1;
    end
end
end