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
%     disp([CII_.maxCII, CII_.minCII])
%     disp([CII_.maxIdx, CII_.minIdx])
    
    
    % Calculate the range of CII (rCII)
    CII_.rCII = max(CII_.CiiValue)-min(CII_.CiiValue);
    
    
    % save 
    CII{ii} = CII_;
    
    
end


%% RAL2022 Figure Presets 
textSize.label = 17;
textSize.axis = 13;
textSize.legend = 14;
textSize.sublabel = 15;
textSize.title = 17;


%% RAL2022 (Revised) CII distribution plot 
% Get meshgrid and array of q2, q3 that were used to generate test
% configurations. 
[qTest, mq2, mq3, aq2, aq3] = getSquatConfig(RoM);
nGrid = size(mq2, 1);   % grid size

figure()
tiledlayout(1,2,'TileSpacing','compact');

% Plot 1 - single heat map
nexttile()

% create colormap (red-white-blue)
nColorSteps = 400;
paramSpan = 6;
cmap = remapCmap(nColorSteps, paramSpan);

% sample the colormap such that white region represents zero.
jj = 5;
data = CII{jj}.CiiValue;
cmapSampled = sampleCmap(cmap, data);
colormap(cmapSampled)

% Plot 1 - surface plot
s = surf(mq2*180/pi, mq3*180/pi, reshape(CII{jj}.CiiValue, nGrid, nGrid));

% Plot 1 - attributes
s.EdgeColor = "none";
cb = colorbar;
% set(cb, 'Direction','reverse')
set(cb,'location', 'eastoutside')
set(cb,'Ticks',[-.05, 0, .2, .3])
set(cb,'TickLabels',[])
limx = [min(aq2), max(aq2)]*180/pi;
limy = [min(aq3), max(aq3)]*180/pi;
xlim(limx)
ylim(limy)
ax = gca;
% set(ax, 'ydir', 'reverse' )
ax.YAxis.FontSize = textSize.axis;
ax.XAxis.FontSize = textSize.axis;
xlabel('HAA, deg', 'FontSize', textSize.sublabel)
ylabel('HFE, deg', 'FontSize', textSize.sublabel)
title('CII of Tello (ColA)', 'FontSize', textSize.sublabel, 'FontWeight', 'normal')
view([0, 0, 1])

% Plot 2
nexttile()
for jj = [3,5]
    
    % colormap
    colormap(cmapSampled)
    
    % plot - surfaces
    s = surf(mq2*180/pi, mq3*180/pi, reshape(CII{jj}.CiiValue, nGrid, nGrid));
    hold on
    
    % attributes
    s.EdgeColor = "none";
    alpha 0.7
    
    % plot - outlines
    CIImtx = reshape(CII{jj}.CiiValue, nGrid, nGrid);
    plot3(aq2*180/pi, aq3(end)*ones(size(aq3))*180/pi, CIImtx(end,:),'k')
    plot3(aq2*180/pi, aq3(1)*ones(size(aq3))*180/pi, CIImtx(1,:),'k')
    plot3(aq2(1)*ones(size(aq2))*180/pi, aq3*180/pi, CIImtx(:,1),'k')
    plot3(aq2(end)*ones(size(aq2))*180/pi, aq3*180/pi, CIImtx(:,end),'k')
    
end

% attributes
xticks([-50, -25, 0, 25, 50])
yticks([-60, -40, -20, 0])
xlim(limx)
ylim(limy)
zlim([-.5, .14])
ax = gca;
ax.YAxis.FontSize = textSize.axis;
ax.XAxis.FontSize = textSize.axis;
ax.ZAxis.FontSize = textSize.axis;
xlabel('HAA, deg', 'FontSize', textSize.sublabel)
ylabel('HFE, deg', 'FontSize', textSize.sublabel)
zlabel('CII', 'FontSize', textSize.sublabel)
view([    38.0101   18.2359])

% export
set(gcf, 'Position', [100, 100, 700, 300])
exportgraphics(gcf, 'RAL2022_CII_Dist_revised.emf','ContentType', 'vector')
% exportgraphics(gcf, 'RAL2022_CII_Dist_revised_1.pdf','ContentType', 'vector')


%% CII (Revised) Distribution Plot - 2
% for easier data manipulation, cii value and range of each robot object is
% collected in a single matrix or array
for ii = 1:nRobot
    CiiValueMtx(:, ii) = CII{ii}.CiiValue';
    rCII(ii) = CII{ii}.rCII;
end

figure()

% figure formatting
w1 = 5;
w2 = 4;
wTotal = w1 + w2;
tiledlayout(1, wTotal, 'TileSpacing', 'normal');

% violin plot settings
vWidth = 0.3;
lvAlpha = .4;

% colormap setting
cIndex = floor(256*(log(rCII)/log(10) + 3.5)/3.5);
cMap = parula(256);

% Robot Labels
row1 = {'    Mini',  'Tello', 'Cassie', ' Atlas', ' Tello', 'Hubo'};
row2 = {'Cheetah', '(CA)', '', '(DRC)', '(ColA)', ' Plus'};
robotLabelArray = [row1; row2];
tickLabels1 = (sprintf('%s\\newline%s\n', robotLabelArray{1:8}));
tickLabels2 = (sprintf('%s\\newline%s\n', robotLabelArray{7:end}));
tickLabels3 = (sprintf('%s\\newline%s\n', robotLabelArray{:}));

% Plot1
nexttile(1,[1,w1])
vHandleLeft = violinplot(CiiValueMtx(:,1:4), '', 'Width', vWidth);
for ii = 1:4
    vHandleLeft(ii).ViolinColor = cMap(cIndex(ii), :);
end

% Plot1 attributes
grid on
box on
ax = gca;
ax.XLim = [0.5, 4.5];
ax.XTickLabel = tickLabels1;
ax.XAxis.FontSize = textSize.axis;
ylim_ = [-0.1, 0.03];
ax.YLim = ylim_;
ax.YTick = -0.1:0.025:0.025;
ax.YAxis.FontSize = textSize.axis;
ylabel('CII', 'FontSize', textSize.sublabel)

% Plot2
nexttile(w1+1,[1,w2])
vHandleRight = violinplot(CiiValueMtx(:,4:6), '', 'Width', vWidth);
for ii = 1:3
    vHandleRight(ii).ViolinColor = cMap(cIndex(3+ii), :);
end

% Plot2 attributes
grid on
box on
ax = gca;
ax.XLim = [.5, 3.5];
ax.XAxis.FontSize = textSize.axis;
ax.XTickLabel = tickLabels2;
ax.YLim = ylim_*6;
ax.YLabel.Rotation = 270;
ax.YTick = -0.5:0.1:.6;
ax.YAxisLocation = 'right';
ax.YAxis.FontSize = textSize.axis;
axpos = ax.YLabel.Position;
axpos(1) = axpos(1) +.1;
ax.YLabel.Position = axpos;
ax.YLabel.FontSize = textSize.sublabel;
ylabel('CII', 'FontSize', textSize.sublabel)

% export
set(gcf, 'Position', [100, 100, 800, 300])
exportgraphics(gcf, 'RAL2022_CII_Dist_Revised_2.emf','ContentType', 'vector')
% exportgraphics(gcf, 'RAL2022_CII_Dist.pdf','ContentType', 'vector')


%% RAL2022 (Revised) log(rCII)
% Log Plot Prep
yTicksDef = -4:1;
yTicks = exp(1).^(yTicksDef);
nColorSteps = 7;
temp = linspace(1, 10, nColorSteps);
spacing = temp(1:nColorSteps-1);
inc = [ (10^-4)*spacing, (10^-3)*spacing, (10^-2)*spacing, (10^-1)*spacing, (10^0)*spacing, (10^1)*spacing];
logInc = log(inc)/log(10);
nInc = length(inc);

figure

% plot horizontal grid (minor)
xSpan = [0, 7];
for ii = 1:nInc
    plot(xSpan, logInc(ii)*ones(1,2),'Color', 0.85*ones(1,3),'LineStyle',':')
    hold on
end

% plot horizontal grid (Major)
for ii = 1:4
    plot(xSpan, (-4+ii)*ones(1,2), 'Color', 0.85*ones(1,3),'LineStyle','-')
end

% plot vertical grid
yCoord = [-3.0, .5];
for ii = 1:6
    plot(ii*ones(1,2), yCoord, 'Color', 0.85*ones(1,3),'LineStyle','-')
end

% plot log(rCII)
scatter(1:6, log(rCII)/log(10), 100, cMap(cIndex, :),'filled','MarkerFaceAlpha', 1, 'MarkerEdgeColor', 'k', 'Marker','s')

% Axis Control
box off
set(gca, 'YTick', yTicksDef)
xlim([.5, 6.5])
ylim([-3.3, .5])

% Label
set(gca, 'XTickLabel', robotNameArray2)
ax = gca;
ax.XAxis.FontSize = textSize.axis;
ax.YAxis.FontSize = textSize.axis;
ylabel("$log\:$(rCII)",'Interpreter','latex', 'FontSize', textSize.sublabel)

% Colorbar
h = colorbar;
set(h,  'TickLabels', {''})

% figure settings
set(gcf, 'Position', [100, 100, 725, 190])
exportgraphics(gcf, 'Revised_RAL2022_log_rCII.emf','ContentType', 'vector')
% exportgraphics(gcf, 'RAL2022_log_rCII.pdf','ContentType', 'vector')


%% RAL2022 Figure - Robot Display at q = minmax Config
nRobot = 6;
ulim  = [-1,1];
vCam = [5, 5, 1];

vCamTarget.max = {[0 0 0], [0 0 -0.17], [-.13 0 -0.3], [0 0 .15], [0 0 -0.17], [0 0 -0.2]};
zoom.max = [1, 1, 1.15, 1.3, 1, 1.2];
vCamTarget.min = {[0 0 0], [0 0 -0.2], [-.13 0 -0.3], [0 0 -.05], [0 0 -0.2], [0 0 -0.25]};
zoom.min = [1, 1, 1.3, 1.7, 1, 1.25];

fileNameCii.max = append(append(repmat({'CiiMax'}, 1, nRobot), robotNames), '.pdf');
fileNameCii.min = append(append(repmat({'CiiMin'}, 1, nRobot), robotNames), '.pdf'); 

for ii = 1:nRobot
    figure
    show(robot{ii}.robotStruc, CII{ii}.maxq(6:end), 'Frames', 'off')
    grid off
    set(get(gca, 'Toolbar'),'Visible','off')
    set(gca, 'CameraTarget', vCamTarget.max{ii})
    set(gca, 'CameraPosition',zoom.max(ii)*vCam)
    set(gcf, 'PaperSize', [5 5])
    set(gcf, 'PaperUnits', 'normalized')
    set(gcf, 'PaperPosition', [0 0 1 1])
    saveas(gcf, fileNameCii.max{ii})
    
    figure
    show(robot{ii}.robotStruc, CII{ii}.minq(6:end), 'Frames', 'off')
    grid off
    set(get(gca, 'Toolbar'),'Visible','off')
    set(gca, 'CameraTarget', vCamTarget.min{ii})
    set(gca, 'CameraPosition',zoom.min(ii)*vCam)
    set(gcf, 'PaperSize', [5 5])
    set(gcf, 'PaperUnits', 'normalized')
    set(gcf, 'PaperPosition', [0 0 1 1])
    saveas(gcf, fileNameCii.min{ii})
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



% Tuning Colormap (tanh)
% conventional colormap



function cmapConv = remapCmap(nColorSteps, paramSpan)

cmap = flip(cbrewer('div','RdBu',2*nColorSteps+1,'linear'));    
remap_temp = tanh(linspace(-paramSpan, paramSpan, 2*nColorSteps+1));
remap = (remap_temp +1);
cmapConv = interp1(linspace(0, 2, 2*nColorSteps+1), cmap(:,:), remap);

end


function cmapSampled = sampleCmap(cmapOriginal, dataVector)
nColorSteps = floor(size(cmapOriginal,1)/2);
absMaxVal = max(dataVector);
absMinVal = abs(min(dataVector));
if absMaxVal > absMinVal
    cmapSampled = cmapOriginal(end-nColorSteps-1-floor(absMinVal/absMaxVal*nColorSteps):end, :);
else
    cmapSampled = cmapOriginal(1:nColorSteps+1+floor(absMaxVal/absMinVal*nColorSteps), :);
end

end

