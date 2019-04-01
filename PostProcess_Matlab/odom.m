%odom
% 
%   <!-- Wheel Mounting Positions -->
%   <xacro:property name="wheelbase" value="0.5120" />
%   <xacro:property name="track" value="0.5708" />
%   <xacro:property name="wheel_vertical_offset" value="0.03282" />
% 
%   <!-- Wheel Properties -->
%   <xacro:property name="wheel_length" value="0.1143" />
%   <xacro:property name="wheel_radius" value="0.1651" />
% <!-- IMU Link is the standard mounting position for the UM6 IMU.-->
%   <!-- Can be modified with environment variables in /etc/ros/setup.bash -->
%   <link name="imu_link"/>
%   <joint name="imu_joint" type="fixed">
%     <origin xyz="$(optenv HUSKY_IMU_XYZ 0.19 0 0.149)" rpy="$(optenv HUSKY_IMU_RPY 0 -1.5708 3.1416)" />
%     <parent link="base_link" />
%     <child link="imu_link" />
%   </joint>
%   <gazebo reference="imu_link">
%   </gazebo>
% 
%   <!-- Husky wheel macros -->
%   <xacro:husky_wheel wheel_prefix="front_left">
%     <origin xyz="${wheelbase/2} ${track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
%   </xacro:husky_wheel>
%   <xacro:husky_wheel wheel_prefix="front_right">
%     <origin xyz="${wheelbase/2} ${-track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
%   </xacro:husky_wheel>
%   <xacro:husky_wheel wheel_prefix="rear_left">
%     <origin xyz="${-wheelbase/2} ${track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
%   </xacro:husky_wheel>
%   <xacro:husky_wheel wheel_prefix="rear_right">
%     <origin xyz="${-wheelbase/2} ${-track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
% </xacro:husky_wheel>
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
