wheel_radius=0.1651;%meters
wheelbase=0.555; %meters
i=1;
for i=2:min(length(rearLeftPos),length(lin_x))
DeltaPosRearRightWheel=rearRightPos(i)-rearRightPos(i-1);
DeltaPosRearLeftWheel=rearLeftPos(i)-rearLeftPos(i-1);

DeltaPosRearWheel(i)=(DeltaPosRearLeftWheel+DeltaPosRearRightWheel)/2;
rearVel(i)=(rearRightVel(i)+rearLeftVel(i))*wheel_radius/2; %lin_x from odom

% DeltaPosFrontWheel(i)=(DeltaPosFrontLeftWheel+DeltaPosFrontRightWheel)/2;
frontVel(i)=(frontRightVel(i)+frontLeftVel(i))*wheel_radius/2; %lin_x from odom

DeltaPosFrontRightWheel=frontRightPos(i)-frontRightPos(i-1);
DeltaPosFrontLeftWheel=frontLeftPos(i)-frontLeftPos(i-1);
DeltaPosRightWheel(i)=(DeltaPosFrontRightWheel+DeltaPosRearRightWheel)/2; %rad
DeltaPosLeftWheel(i)=(DeltaPosFrontLeftWheel+DeltaPosRearLeftWheel)/2; %rad
heading(i)=(DeltaPosRightWheel(i)-DeltaPosLeftWheel(i))/(wheelbase); %ang_z from odom
headRate(i)=((rearLeftVel(i)+frontLeftVel(i))/2-(rearRightVel(i)+frontRightVel(i))/2)*wheel_radius/(0.545);
headRateF(i)=(frontLeftVel(i)-frontRightVel(i))/(0.272);
end
