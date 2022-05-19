%% Import URDF files
clear; clc;
addpath(genpath(pwd))


% Define the number and names of robots that will be imported
nRobot = 6;
robotNames = {'MiniCheetah', 'TelloCA', 'Cassie', 'Atlas', 'TelloColA', 'HuboPlus'};


% Filenames of URDF (under 'urdf' folder)
fileNames = append(repmat({'URDF_'},1,nRobot), append(robotNames, '_Float.urdf'));


%% Populate Nominal / Test Configurations
% For all robots, joint angle denoted as:
% q1: Hip rotation (yaw, z-axis)
% q2: Hip ad/abduction (roll, x-axis)
% q3: Hip flexion/extension (pitch, y-axis)
% q4: Knee flexion/extension (pitch, y-axis)
% q5: Ankle flexion/extension (pitch, y-axis)


% Test range of motion (RoM)
RoM.q2 = [-pi*50/180, pi*50/180];
RoM.q3 = [0, -pi/3];


% Populate trajectory of angle vectors for squat motion
% qTest is 3 x N matrix, each column is a configuration, [q2; q3; q4]
[qTest, ~, ~, ~, ~] = getSquatConfig(RoM);


% Nominal Configuration
q0 = zeros(3,1);    % q2 = q3 = q4 = 0;


%% Build Rigid Body Model 
% This portion takes ~30 seconds
% Create place holder for robot instances
robot = cell(1,nRobot);


% Instantiate robot objects
for ii = 1:nRobot  % for all robots
    robot{ii} = RBDyn3(fileNames{ii}, robotNames{ii});  
    disp(['Rigid body structure of ',sprintf(robotNames{ii}), ' is created. (', num2str(ii), '/', num2str(nRobot), ')'])
end


%% Get configurations for each robot
% Both test configuration and nominal configuration vectors are assumed to
% be 3 x 1 (or 3 x N). To conform to full-robot's configuration, those
% vectors need to be expanded according to the robot's joint structure.


qFull0 = cell(1, nRobot);       % create place holders
qFullTest = cell(1, nRobot);


% convert partial configuration vectors to full configuration vectors
for ii = 1:nRobot
    qFull0{ii} = robot{ii}.genFullConfig(q0);            
    qFullTest{ii} = robot{ii}.genFullConfig(qTest);      
end
    

%% Calculate Centroidal Inertia Isotropy (CII)
% CII measures proximodistal distribution of mass for any rigid body
% structure. CII is evaluated for a single configuration with respect to
% nominal configuration. 


CII = cell(1, nRobot);  % Create CII place holders


for ii = 1:nRobot       % for all robots
    robot_ = robot{ii};
    
    
    % Associate nominal configuration with floating base rigid body model (RbmFloat)
    CII_ = robot_.calcCII(qFull0{ii}, qFullTest{ii});
    
    
    % Find configuration where CII is maximum or minimum
    CII_.maxIdx = find(CII_.CiiValue == max(CII_.CiiValue));
    CII_.minIdx = find(CII_.CiiValue == min(CII_.CiiValue));
    CII_.maxq = CII_.config(:, CII_.maxIdx);
    CII_.minq = CII_.config(:, CII_.minIdx);
    CII_.maxCII = max(CII_.CiiValue);
    CII_.minCII = min(CII_.CiiValue);
    
    
    % Calculate the range of CII (rCII)
    CII_.rCII = max(CII_.CiiValue)-min(CII_.CiiValue);
    
    
    % save 
    CII{ii} = CII_;
    
    
end


%% Visualize Robot (This section requires Matlab Robotics Toolbox)
% To visualize, three joint angles need to be defined: 
% q = [q2, q3, q4]      q2: HAA, q3: HFE, q4: KFE

% For example, 
% q = [0; 0; 0];        % nominal pose
% q = [pi/4; 0; 0];     % legs spread open
% q = [0; pi/4; 0];     % hip pitch backward
% q = [0; 0; pi/4];     % knee pitch backward
% q = [0; -pi/4; pi/2]; % knee bent


% Visualize a single robot 
q1 = zeros(1,3);
q2 = [pi/4, -pi/3, 2*pi/3];
robotVis = robot{4};    % 4 = Atlas DRC
figure()
robotVis.showRobot(robotVis.genFullConfig(q1))    % visualization

% Visualize a single robot at two different configurations
figure()
subplot(1,2,1)
robotVis.showRobot(CII{4}.maxq)    % visualization
title('maxpose')
subplot(1,2,2)
robotVis.showRobot(CII{4}.minq)    % visualization
title('minpose')

% Visualize all robots in configurations where CII is maximum or minimum
figure()
for ii = 1:nRobot
    subplot(2,6,ii)
    robot{ii}.showRobot(CII{ii}.minq)
    subplot(2,6, ii+6)
    robot{ii}.showRobot(CII{ii}.maxq)
end


function [q, mq2, mq3, aq2, aq3] = getSquatConfig(RoM)
% q1: Hip rotation (yaw, z-axis)
% q2: Hip ad/abduction (roll, x-axis)
% q3: Hip flexion/extension (pitch, y-axis)
% q4: Knee flexion/extension (pitch, y-axis)
% q5: Ankle flexion/extension (pitch, y-axis)
q2 = RoM.q2;
q3 = RoM.q3;
N = 50;


% create meshgrid from q2 and q3
aq2 = linspace( min(q2), max(q2), N);
aq3 = linspace( min(q3), max(q3), N);
[mq2, mq3] = meshgrid(aq2, aq3);

% vectorize the meshgrid
q2 = mq2(:)';
q3 = mq3(:)';
q4 = -2*q3; % asserts squating motion
q = [q2; q3; q4];
end

