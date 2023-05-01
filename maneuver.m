function [Iw,phi,H,T,t] = maneuver (Omega_max,t_maneuver,Is,man_angle,beta,mode,Iw)

phi_final=0;
n=1;

while phi_final<man_angle
    if nargin== 5 
        Iw_vec=3:0.1:10;
        Iw=Iw_vec(n);
    end
    
    if mode=='Roll' || mode=='Pitch'
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
    elseif mode=='Yaw'
        
    end
    n=n+1;
end
end