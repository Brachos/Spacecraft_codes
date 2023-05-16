function [phi,Iw,H,T,t,z] = maneuver (Omega_max,t_maneuver,Is,beta,phi_man,mode)

if mode=='Roll' || mode=='Pitch'
        % Phase 1
        t1=0:0.1:t_maneuver/2;
        zdot1=2*phi_man/(t1(end)^2);
        z1=zdot1*t1;
        Iw=z1(end)*Is/(2*Omega_max*sin(beta));
        phi1=1/2*zdot1*t1.^2;
        T1=Is*zdot1*ones(1,length(t1));

        % Phase 2
        t2=0:0.1:t_maneuver/2;
        zdot2=-2*phi_man/t2(end)^2;%add a minus because here the wheels decelerate-->acceleration is negative
        z2=zdot2*t2;
        phi2=1/2*zdot2*t2.^2+z1(end).*t2+phi1(end);
        T2=Is*zdot2*ones(1,length(t2)); 

        % Total
        t=[t1 t2+t_maneuver/2];
        phi=[phi1 phi2];
        z=[z1 z2+z1(end)];
        H= Is*z; %angular momentum of the spacecraft.
        T= [T1 T2];
        
elseif mode=='Yaw'
        yaw_torque = 4000; %[Nm]
        yaw_torque_time = 0.5; %[s]
        
        %Phase 1, the objective is to find the velocity and position of the
        %spacecraft after 0.5s.
        zdot1=yaw_torque/Is;
        t1=0:0.1:yaw_torque_time;
        z1=zdot1.*t1; 
        phi1=1/2*zdot1*t1.^2; % rotation angle of the spacecraft over 0.5s
        T1=yaw_torque*ones(1,length(t1));
        
        %Phase 2 
        t2=0:0.1:t_maneuver/2;
        zdot2=2*phi_man/t2(end)^2;
        z2=zdot2*t2;
        Iw=z2(end)*Is/(4*Omega_max*cos(beta));
        phi2=-1/2*zdot2*t2.^2+phi1(end);
        T2=Is*zdot2*ones(1,length(t2));
        
        %Phase 3
        t3=0:0.1:t_maneuver/2;
        zdot3=-2*phi_man/t2(end)^2;
        z3=zdot3*t3;
        phi3=-1/2*zdot3*t3.^2-z2(end).*t3+phi2(end);
        T3=Is*zdot3*ones(1,length(t3));
        
        % Total
        t=[t1 t2+t1(end) t3+t_maneuver/2+t1(end)];
        phi=[phi1 phi2 phi3];
        z=[z1 z2+z1(end) z3+z2(end)+z1(end)];
        H= Is*z; %angular momentum of the spacecraft.
        T= [T1 T2 T3];
        

end
end