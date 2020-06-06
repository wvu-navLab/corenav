%% Differential Wheel Rover Low Level Navigation post processing script
% Authors: Cagri Kilic and Jason Gross
% Date: September 2018
%%  -----------------------------------------------------------------------
% clc;
clear;
load LongRunData.mat % LshapeData.mat %ForwardDrive.mat
%% odometry calculations
odom
%% initialize variables
LongRunInit % LshapeInit.mat %ForwardDriveInit.mat

%% Double Low Pass Filter
dema
for i=2:L
    dtIMU=tTimuAdis(i)-tTimuAdis(i-1); % calculates imu delta time from rostomat output tTimu
    TimeIMU(i)=dtIMU+TimeIMU(i-1); % creates imu time from delta time starting from zero
    %%  -------------------------------------------------------------------
    %% IMU mounting orientation for BadgerRover
    omega_b_ib=[-Gy(i),Gx(i),Gz(i)]-bg(:,i-1)'; %rad/s IMU gyro outputs minus estimated gyro bias
    f_ib_b = [-Ay(i),Ax(i),Az(i)]-ba(:,i-1)'; %m/s^2 IMU acceleration minus estimated acce bias
    v_ib_b= f_ib_b* dtIMU; %m/s acceleration times delta IMU = velocity
    %% --------------------------------------------------------------------
    %% Attitude Update
    [insAttPlus, Cb2nPlus,Cb2nMinus,Omega_n_en,Omega_n_ie,R_N,R_E,omega_n_in,omega_n_ie] = AttUpdate(insAtt(:,i-1),omega_ie,insLLH(:,i-1),omega_b_ib,ecc,Ro,insVel(:,i-1),dtIMU);
    insAtt(:,i)=insAttPlus;
    %% Velocity Update
    [insVelPlus] = VelUpdate(Cb2nMinus, Cb2nPlus, v_ib_b,insVel(:,i-1),insLLH(:,i-1),omega_ie,Ro,ecc,dtIMU);
    insVel(:,i)=insVelPlus;
    %% Position Update
    [insLLHPlus,R_EPlus,r_eb_e] = PosUpdate(insLLH(:,i-1),insVel(:,i),insVel(:,i-1),Ro,ecc,dtIMU);
    insLLH(:,i)=insLLHPlus;
    insXYZ(:,i)=r_eb_e;
    %% --------------------------------------------------------------------
    %% Error State Model--eq 14.63
    [STM] = insErrorStateModel_LNF(R_EPlus,R_N,insLLH(:,i),insVel(:,i),dtIMU,Cb2nPlus,omega_ie,omega_n_in,f_ib_b);
    %% Propagation of the Error
    x_err=STM*x_err;
    %% Q matrix --
    F21= -skewsymm(Cb2nPlus*(f_ib_b'));
    Q=getQins(F21,Cb2nPlus,insLLH(:,i),R_N,R_E,dtIMU);
    [~,q] = chol(Q);
    if q ~= 0
        disp('Q matrix is not positive definite')
        i
    end
    %% P matrix
    P = STM*P*STM' + Q;
    [~,p] = chol(P);
    if p ~= 0
        disp('P matrix is not positive definite')
        i
    end
    if odomUpdate()
        %% --------------------------------------------------------------------
        Cn2bPlus=Cb2nPlus';
        omega_b_ie=Cn2bPlus*omega_n_ie;
        omega_b_ei=-omega_b_ie;
        omega_b_eb=omega_b_ei+omega_b_ib';
        %% Measurement Matrix -- integration part for INS -- eq 16.48
        H11= H11+ [1,0,0]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
        H12= H12+ [1,0,0]*Cn2bPlus*dtIMU;
        H21= H21+ sin(insAtt(2,i))*[0,cos(insAtt(1,i)),sin(insAtt(1,i))]*Cn2bPlus*dtIMU;
        H31= H31+ [0,1,0]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
        H32= H32+ [0,1,0]*Cn2bPlus*dtIMU;
        H41= H41+ [0,0,1]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
        H42= H42+ [0,0,1]*Cn2bPlus*dtIMU;
        %% Measurement Innovation -- integration part for INS -- eq 16.42
        z11=z11+[1,0,0]*(Cn2bPlus*insVel(:,i) + skewsymm(omega_b_eb)*[-A,0,0]')*dtIMU;
        z21=z21+cos(insAtt(2,i))*dtIMU;
        z31=z31+[0,1,0]*(Cn2bPlus*insVel(:,i) + skewsymm(omega_b_eb)*[-A,0,0]')*dtIMU;
        z41=z41+[0,0,1]*(Cn2bPlus*insVel(:,i) + skewsymm(omega_b_eb)*[-A,0,0]')*dtIMU;
        % z and H values are integrated for IMU only above. When the Odometry
        % update is available (tTimu(i)>=tTodom(kk)) integrated values will be
        % updated by odometry measurements, used in the filter and destroyed.
    end
    if nonHolo()
        [insAtt(:,i),insVel(:,i),insLLH(:,i),x_err,P]= nonHolonomic(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,omega_n_ie,omega_b_ib,A);
    end
    %% --------------------------------------------------------------------
    xState{1,i}=x_err;
    PStore{1,i}=P;
    STMStore{1,i}=STM+eye(15).*x_err;
    if kk<min(min(length(heading),length(lin_x))) % Only valid for post-processing
        
        %% ----------------------------------------------------------------
        if tTimu(i)>=tTodom(kk) % Odometry update is available
            bb(counter)=i; % counts for the number of `i` when the loop goes into odometry updates
            dt_odom=tTodom(kk)-tTodom(kk-1);
            %% ------------------------------------------------------------
            %% Odometry Update
            if odomUpdate()
                odomUptCount=odomUptCount+1;
                
                P_old=P;
                insAtt_old= insAtt(:,i);
                insVel_old= insVel(:,i);
                insLLH_old= insLLH(:,i);
                x_err_old=x_err;
                [insAtt(:,i),insVel(:,i),insLLH(:,i),x_err,P,postFitOdom]= odomUpt(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,...
                    insAtt(:,i-1),insAtt(3,bb(counter-1)),dt_odom,rearVel(kk),headRate(kk),s_or,...
                    H11,H12,H21,H31,H32,H41,H42,z11,z21,z31,z41,T_r,...
                    sigma_or_L,sigma_or_R,sigma_cmc,s_delta_or);
                odompostfit(:,odomUptCount)=postFitOdom.z-postFitOdom.H*x_err;
                S=postFitOdom.H*P*postFitOdom.H'+[0.00045,0,0,0;0,0.1152,0,0;0,0,0.0025,0;0,0,0,0.0025];
                chisq(:,odomUptCount)=odompostfit(:,odomUptCount)'*inv(S)*odompostfit(:,odomUptCount);
                mahala(:,odomUptCount)=sqrt(odompostfit(:,odomUptCount)'*inv(S)*odompostfit(:,odomUptCount));
                
                if mahala(1,odomUptCount)>5
                    
                    %                     insAtt(:,i)=insAtt_old;
                    %                     insVel(:,i)=insVel_old;
                    %                     insLLH(:,i)=insLLH_old;
                    %                     P=P_old;
                    %                     x_err=x_err_old;
                    LLHcorrected2(:,cttr2)=insLLH(:,i);
                    cttr2=cttr2+1;
                end
                slipR(:,odomUptCount)=(rearVel(odomUptCount)-sqrt(insVel(1,i)^2+insVel(2,i)^2))/rearVel(odomUptCount);
                if slipR(:,odomUptCount) < -50
                    slipR (:,odomUptCount) = 0;
                end
                if abs(slipR(1,odomUptCount))>0.25
                    LLHcorrected1(:,cttr0)=insLLH(:,i);
                    cttr0=cttr0+1;
                end
            end % odom update
            % % % ------------------------------------------------------------
            % destroy H and z values for the next values
            H11=zeros(1,3);
            H12=zeros(1,3);
            H21=zeros(1,3);
            H31=zeros(1,3);
            H32=zeros(1,3);
            H24=zeros(1,3);
            H41=zeros(1,3);
            H42=zeros(1,3);
            z11=0;
            z21=0;
            z31=0;
            z41=0;
            counter=counter+1;
            kk=kk+1;
        end % odom update not available
        %% ----------------------------------------------------------------
        
        if abs(lin_x(kk))<0.04% triggers zupt
            zeroUptCount=zeroUptCount+1;
            if zeroUpdate()
                [insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,postFitZero] = zeroUpd(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,omega_b_ib);
            end
            zCtr(i)=cttr3+offsetCtr;
            LLHcorrected(:,cttr3)=insLLH(:,i);
            cttr3=cttr3+1;
        else
            zCtr(i)=0;
            offsetCtr=offsetCtr+1;
        end % zero update not available
        
        if backProp()
            if zCtr(i)-zCtr(i-1) < 0 % checks the zero update counter diff. If it is > 0 ZeroUpdate applied
                doBackProp=false;
                for j=i-1:-1:2
                    if zCtr(j)-zCtr(j-1)<0
                        doBackProp=true;
                        lastZindex=j;
                        break;
                    end
                end
                if doBackProp
                    x_err_s=xState{1,i};
                    P_s=PStore{1,i};
                    STM_s=STMStore{1,i};
                    for dd=i:-1:lastZindex
                        [P_s,x_err_s] = smoothback(PStore{dd-1},PStore{dd},STMStore{dd-1},xState{dd-1},xState{dd},x_err_s,P_s,STM_s);
                        insVel(:,dd)=insVel(:,dd)-x_err_s(4:6);
                        insLLH(:,dd)=insLLH(:,dd)-x_err_s(7:9);
                        Cn2b_propBack= eulr2dcm(insAtt(:,dd));
                        insAtt(:,dd)=dcm2eulr((eye(3)-skewsymm(x_err_s(1:3)))*Cn2b_propBack');
                        ba(1:3,dd)=x_err_s(10:12);
                        bg(1:3,dd)=x_err_s(10:12);
                    end
                else
                    ba(1:3,i)=x_err(10:12); % acce bias, this value will be removed from IMU acce output
                    bg(1:3,i)=x_err(13:15); % gyro bias, this value will be removed from IMU gyro output
                end % doBackProp
            end % if ZeroUpdate applied
        else
            %                     ba(1:3,i)=x_err(10:12); % acce bias, this value will be removed from IMU acce output
            %                     bg(1:3,i)=x_err(13:15);
        end
    end
    
    sig1(i)=3*sqrt(abs(P(1,1))); % 3 sigma values of att_x -roll
    sig2(i)=3*sqrt(abs(P(2,2))); % 3 sigma values of att_y -pitch
    sig3(i)=3*sqrt(abs(P(3,3))); % 3 sigma values of att_z -yaw
    
    sig4(i)=3*sqrt(abs(P(4,4))); % 3 sigma values of vel_x -forward
    sig5(i)=3*sqrt(abs(P(5,5))); % 3 sigma values of vel_y -left
    sig6(i)=3*sqrt(abs(P(6,6))); % 3 sigma values of vel_z -down
    
    sig7(i)=3*sqrt(abs(P(7,7))); % 3 sigma values of pos_x -latitude
    sig8(i)=3*sqrt(abs(P(8,8))); % 3 sigma values of pos_y -longitude
    sig9(i)=3*sqrt(abs(P(9,9))); % 3 sigma values of pos_z -height
    if gpsResults()
        gpsLonger(:,i)=[llhGPS(1,kk);llhGPS(2,kk);llhGPS(end,kk)];
    end
    x_State(:,i)=[insAtt(:,i);insVel(:,i);insLLH(:,i);ba(:,i);bg(:,i)];
    Cn2b_corr= eulr2dcm(insAtt(:,i));
    insAttCorr(:,i)=dcm2eulr((eye(3)-skewsymm(x_err(1:3)))*Cn2b_corr');
    insVelCorr(:,i)=insVel(:,i)-x_err(4:6);
    insLLHCorr(:,i)=insLLH(:,i)-x_err(7:9);
    
    
if contactAngle()
pitch=insAtt(2,i)*180/pi;
pitch_rate=((insAtt(2,i)-insAtt(2,i-1))*180/pi)/dtIMU;
wheelCenterSpeedFront=frontRightVel(kk)*0.1651;
wheelCenterSpeedBack=rearLeftVel(kk)*0.1651;

% if abs(pitch_rate)<1 && wheelCenterSpeedFront==0 && wheelCenterSpeedBack==0 % (pg:41)
if wheelCenterSpeedFront==0 && wheelCenterSpeedBack==0 % (pg:41)

%   disp('Robot is stationary, infinite set of angles')
  x_gamma(:,i)=x_gamma(:,i-1);
else
  x_gamma(:,i)=x_gamma(:,i-1);
  R_gamma=diag([0.001^2, 0.001^2]);
  P_gamma=P_gamma+R_gamma;
  [x_gamma(:,i),P_gamma]=wheelTerrainContactAngle(pitch,pitch_rate,wheelCenterSpeedFront,wheelCenterSpeedBack,x_gamma(:,i-1),P_gamma);
  psig1(i)=P_gamma(1,1);
  psig2(i)=P_gamma(2,2);
  if cos(x_gamma(2,i)-pitch)==0
      if wheelCenterSpeedFront==-wheelCenterSpeedBack
        disp('Pure Rotation Occurred')
        x_gamma(1,i)=pitch+pi/2 * sign(pitch_rate);
        x_gamma(2,i)=pitch-pi/2 * sign(pitch_rate);
      elseif pitch_rate==0 && wheelCenterSpeedFront==wheelCenterSpeedBack
        disp('Pure Translation Occurred, using previous solution')
        x_gamma(:,i)=x_gamma(:,i-1);
      end
    end
end
% end
  psig1(i)=P_gamma(1,1);
  psig2(i)=P_gamma(2,2);
end    
    
end
figureGeneration
