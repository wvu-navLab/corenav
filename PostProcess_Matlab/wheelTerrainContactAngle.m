function[x_new,P_new]=wheelTerrainContactAngle(p,pr,vf,vb,x_old,P_old)
l=0.5; %m distance between wheels

a=l*pr/vf;
b=vb/vf;

if (2*a^2+2*b^2+2*a^2*b^2-a^4-b^4-1)>=0
    h=(1/(2*a))*sqrt(2*a^2+2*b^2+2*a^2*b^2-a^4-b^4-1);
    
    gamma1=p-acos(h);%contact angle 1
    gamma2=acos(h/b)+p; %contact angle 2
    if isreal(gamma1) || isreal(gamma2)
        
        dyz=[1,   -(pr^4/16 - vb^4 + 2*vb^2*vf^2 - vf^4)/(pr^2*vf^3*((- pr^4/16 + (pr^2*vb^2)/2 + (pr^2*vf^2)/2 - vb^4 + 2*vb^2*vf^2 - vf^4)/vf^4)^(1/2)*((pr^2/4 - vb^2 + vf^2)^2/(pr^2*vf^2))^(1/2)), -(- pr^4/16 + (pr^2*vb^2)/2 - vb^4 + vf^4)/(pr*vf^4*((- pr^4/16 + (pr^2*vb^2)/2 + (pr^2*vf^2)/2 - vb^4 + 2*vb^2*vf^2 - vf^4)/vf^4)^(1/2)*((pr^2/4 - vb^2 + vf^2)^2/(pr^2*vf^2))^(1/2)),              (4*vb*(pr^2/4 - vb^2 + vf^2))/(pr*vf^3*((- pr^4/16 + (pr^2*vb^2)/2 + (pr^2*vf^2)/2 - vb^4 + 2*vb^2*vf^2 - vf^4)/vf^4)^(1/2)*((4*(pr^2/4 - vb^2 + vf^2)^2)/(pr^2*vf^2))^(1/2));
            1, (pr^4/16 - vb^4 + 2*vb^2*vf^2 - vf^4)/(pr^2*vb*vf^2*((- pr^4/16 + (pr^2*vb^2)/2 + (pr^2*vf^2)/2 - vb^4 + 2*vb^2*vf^2 - vf^4)/vf^4)^(1/2)*((pr^2/4 + vb^2 - vf^2)^2/(pr^2*vb^2))^(1/2)),               -(pr^2 + 4*vb^2 - 4*vf^2)/(2*pr*vb*vf*((- pr^4/16 + (pr^2*vb^2)/2 + (pr^2*vf^2)/2 - vb^4 + 2*vb^2*vf^2 - vf^4)/vf^4)^(1/2)*((pr^2/4 + vb^2 - vf^2)^2/(pr^2*vb^2))^(1/2)), (- pr^4/16 + (pr^2*vf^2)/2 + vb^4 - vf^4)/(pr*vb^2*vf^2*((- pr^4/16 + (pr^2*vb^2)/2 + (pr^2*vf^2)/2 - vb^4 + 2*vb^2*vf^2 - vf^4)/vf^4)^(1/2)*((pr^2/4 + vb^2 - vf^2)^2/(pr^2*vb^2))^(1/2))];
        
        R=diag([3^2, 3^2, 0.05^2, 0.05^2]);
        
        K=P_old*inv(P_old+dyz*R*dyz');
        
        x_new=x_old+K*([gamma1;gamma2]-x_old);
        
        P_new=(eye(2)- K)*P_old;
    else
        x_new=x_old;
        P_new=P_old;
    end
else
    x_new=x_old;
    P_new=P_old;
end
