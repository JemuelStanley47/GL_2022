%% P. JEMUEL STANLEY
% Dt: 09-04-2021 14:07 (DD-MM-YY HH-MM)
% Reference: https://in.mathworks.com/help/robotics/ug/interactively-build-a-trajectory-abb-yumi.html
% https://in.mathworks.com/help/robotics/ref/analyticalinversekinematics.html

clear all
clc
close all

%% Load and display robot
%robot = loadrobot('abbYuMi', 'Gravity', [0 0 -9.81]);
%robot = loadrobot('abbIrb120','DataFormat','row')
robot  = loadrobot('abbYuMi','DataFormat','row');

%% END-EFFECTOR CARTESIAN POSES

% __________________J1__________________
%quat = [0.13270,0.68243,-0.71691,0.05216];
%trans = [0.47636,-0.11326,0.02930];
% __________________J2__________________
%quat = [0.13270,0.68243,-0.71691,0.05216];
%trans = [0.47255,0.32076,0.03726];
% __________________J3__________________
%quat = 
%trans=
% __________________J4__________________
%quat = 
%trans=
% __________________JM__________________
quat = [0.00711,0.48559,-0.87344,0.03543]; % JM
trans = [0.33217,0.10627,0.02548]; % JM
% __________________J12__________________
%quat = [0.08156,0.61512,0.78287,-0.04578]; % J12
%trans = [0.48485,0.10626,0.03509]; % J12
% __________________J23__________________
%quat = [0.01703,0.99831,-0.04764,-0.02843];
%trans= [0.32853,0.31773,0.02832];
% __________________J14__________________
%quat = [0.06370,0.67216,0.71980,-0.16134];
%trans= [0.32953,-0.10979,0.03106];
time_start = cputime;

rotm = quat2rotm(quat);
tform = rotm2tform(rotm);
tform(1,4) = trans(1);
tform(2,4) = trans(2);
tform(3,4) = trans(3);

%robot = 'abbYumi'
%load_system(robot)
%show(robot)
% aik = analyticalInverseKinematics(robot);
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.25 0.25 0.25 1 1 1];
initialguess = robot.homeConfiguration;
%show(robot, initialguess)
%randConfig = robot.randomConfiguration;
%tform = getTransform(robot,randConfig,'gripper_l_finger_r','yumi_body')

[configSoln,solnInfo] = ik('gripper_l_finger_r',tform,weights,initialguess);
joints = [];

c = [];
for i=1:7
    joints(i)=configSoln(i)*180/pi;
    if (i==1) || (i==4) || (i==6)
        if joints(i)>=-45 && joints(i)<45
            c = [c 0];
        end
        if joints(i)>=45 && joints(i)<135
            c = [c 1];
        end
        if joints(i)>=135 && joints(i)<225
            c = [c 2];
        end
        if joints(i)>=225 && joints(i)<315
            c = [c 3];
        end
        if joints(i)<=-45 && joints(i)>-135
            c = [c -1];
        end
        if joints(i)<=-135 && joints(i)>-225
            c = [c -2];
        end
        if joints(i)<=-225 && joints(i)>-315
            c = [c -3];
        end
        if joints(i)<=-315 && joints(i)>-360
            c = [c -4];
        end
    end
end
c(4) = 0;
time_stop = cputime - time_start;
disp(time_stop)
disp(joints)
disp(c)
show(robot,configSoln);


