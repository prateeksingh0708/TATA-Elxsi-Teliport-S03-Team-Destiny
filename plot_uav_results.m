% plot_uav_results.m
% Generates all 6 analysis plots from the latest simulation log.
% Run with: >> plot_uav_results
% No arguments needed - auto-loads the latest saved log file.

close all;

%% ===================== LOAD LATEST LOG =====================
homeDir = getenv('HOME');
if isempty(homeDir), homeDir = getenv('USERPROFILE'); end
save_dir  = fullfile(homeDir,'Desktop','UAV_Landing_Project','Data','simulation_logs');
mat_files = dir(fullfile(save_dir,'*.mat'));

if isempty(mat_files)
    fprintf('ERROR: No log files found in:\n  %s\n', save_dir);
    fprintf('Run the simulation first:  >> run_simulation_animated\n');
    return;
end

[~, newest] = max([mat_files.datenum]);
fpath = fullfile(save_dir, mat_files(newest).name);
fprintf('Loading: %s\n', fpath);
S = load(fpath);
d = S.simlog;
fprintf('Loaded: %d steps, %.1f s\n\n', length(d.time), d.time(end));

%% Shared variables
T  = d.time;
N  = length(T);
obstacle_pos    = [25, 1, -3];
obstacle_radius = 1.5;
Lp = 8; Wp = 2.5;

%% ===================== FIGURE 1: 3D TRAJECTORY =====================
figure('Name','Fig 1: 3D Trajectory','Position',[50 50 1200 800]);
hold on; grid on; axis equal; view(45,25);

plot3(d.drone_pos(1,:),    d.drone_pos(2,:),    -d.drone_pos(3,:), ...
    'b-','LineWidth',2.5,'DisplayName','Drone path');
plot3(d.platform_pos(1,:), d.platform_pos(2,:), -d.platform_pos(3,:), ...
    'r--','LineWidth',2,'DisplayName','Platform path');

[Xs,Ys,Zs] = sphere(30);
surf(obstacle_pos(1)+obstacle_radius*Xs, ...
     obstacle_pos(2)+obstacle_radius*Ys, ...
    -obstacle_pos(3)+obstacle_radius*Zs, ...
    'FaceColor',[1 0.5 0],'FaceAlpha',0.4,'EdgeColor','none','DisplayName','Obstacle');

gray = [0.8 0.8 0.8; 0.6 0.6 0.6; 0.4 0.4 0.4; 0.2 0.2 0.2];
for k = 1:4
    idx = max(1, floor((k-1)*(N-1)/3)+1);
    pp  = d.platform_pos(:,idx);
    cx  = [pp(1)+Lp/2, pp(1)+Lp/2, pp(1)-Lp/2, pp(1)-Lp/2];
    cy  = [pp(2)+Wp/2, pp(2)-Wp/2, pp(2)-Wp/2, pp(2)+Wp/2];
    cz  = repmat(-pp(3),1,4);
    patch(cx,cy,cz,gray(k,:),'FaceAlpha',0.4,'EdgeColor','k','LineWidth',1.2);
end

plot3(d.drone_pos(1,1),   d.drone_pos(2,1),   -d.drone_pos(3,1), ...
    'go','MarkerSize',14,'LineWidth',3,'DisplayName','Start');
plot3(d.drone_pos(1,end), d.drone_pos(2,end), -d.drone_pos(3,end), ...
    'r*','MarkerSize',18,'LineWidth',3,'DisplayName','Landing');

phase_idx = [1];
for k = 2:N
    if ~strcmp(d.phase{k}, d.phase{k-1})
        phase_idx(end+1) = k;
    end
end
for k = 1:length(phase_idx)
    ii = phase_idx(k);
    text(d.drone_pos(1,ii), d.drone_pos(2,ii), -d.drone_pos(3,ii), ...
        ['  ' upper(d.phase{ii})], 'FontSize',9,'FontWeight','bold','Color','blue');
end

xlabel('North (m)','FontSize',12,'FontWeight','bold');
ylabel('East (m)', 'FontSize',12,'FontWeight','bold');
zlabel('Altitude (m)','FontSize',12,'FontWeight','bold');
title('3D Trajectory - UAV Landing on Moving Platform','FontSize',14,'FontWeight','bold');
legend('Location','best','FontSize',10);

%% ===================== FIGURE 2: POSITION ERROR =====================
figure('Name','Fig 2: Position Error','Position',[100 100 1400 500]);

subplot(1,4,1);
plot(T, d.pos_error(1,:),'r-','LineWidth',1.5);
hold on; grid on; yline(0,'k--');
xlabel('Time (s)'); ylabel('Error (m)');
title('X (North) Error','FontWeight','bold');

subplot(1,4,2);
plot(T, d.pos_error(2,:),'g-','LineWidth',1.5);
hold on; grid on; yline(0,'k--');
xlabel('Time (s)'); ylabel('Error (m)');
title('Y (East) Error','FontWeight','bold');

subplot(1,4,3);
plot(T, d.pos_error(3,:),'b-','LineWidth',1.5);
hold on; grid on; yline(0,'k--');
xlabel('Time (s)'); ylabel('Error (m)');
title('Z (Down) Error','FontWeight','bold');

subplot(1,4,4);
plot(T, vecnorm(d.pos_error),'k-','LineWidth',2);
hold on; grid on;
yline(0.5,'r--','LineWidth',1.5,'DisplayName','0.5m target');
xlabel('Time (s)'); ylabel('Total Error (m)');
title('Total 3D Error','FontWeight','bold');
legend('Location','best');

sgtitle('Position Error Convergence','FontSize',14,'FontWeight','bold');

%% ===================== FIGURE 3: VELOCITY SYNC =====================
figure('Name','Fig 3: Velocity Sync','Position',[150 150 1200 700]);
vlabels = {'Vx North (m/s)','Vy East (m/s)','Vz Down (m/s)'};
for s = 1:3
    subplot(3,1,s); hold on; grid on;
    plot(T, d.drone_vel(s,:),    'b-','LineWidth',2,'DisplayName','Drone');
    plot(T, d.platform_vel(s,:), 'r--','LineWidth',2,'DisplayName','Platform');
    ylabel(vlabels{s},'FontWeight','bold');
    legend('Location','best');
    if s == 3, xlabel('Time (s)'); end
end
sgtitle('Velocity Matching - Drone vs Platform','FontSize',14,'FontWeight','bold');

%% ===================== FIGURE 4: WIND ANALYSIS =====================
figure('Name','Fig 4: Wind','Position',[200 200 1400 600]);
wm = vecnorm(d.wind_vel);

subplot(2,2,1);
plot(T,wm,'k-','LineWidth',1.5); hold on; grid on;
plot(T,movmean(wm,50),'r-','LineWidth',2,'DisplayName','50-step avg');
xlabel('Time (s)'); ylabel('Wind speed (m/s)');
title('Wind Magnitude','FontWeight','bold'); legend('Location','best');

subplot(2,2,2);
plot(T, vecnorm(d.pos_error),'b-','LineWidth',1.5); hold on; grid on;
xlabel('Time (s)'); ylabel('Position error (m)');
title('Position Error Over Time','FontWeight','bold');

subplot(2,2,3); hold on; grid on;
plot(T, d.wind_vel(1,:),'r-','LineWidth',1.2,'DisplayName','Wx');
plot(T, d.wind_vel(2,:),'g-','LineWidth',1.2,'DisplayName','Wy');
plot(T, d.wind_vel(3,:),'b-','LineWidth',1.2,'DisplayName','Wz');
xlabel('Time (s)'); ylabel('Wind (m/s)');
title('Wind Components','FontWeight','bold'); legend('Location','best');

subplot(2,2,4);
histogram(wm,30,'Normalization','probability','FaceColor',[0.3 0.5 0.8]);
hold on; grid on;
xline(mean(wm),'r--','LineWidth',2,'Label','Mean');
xlabel('Wind speed (m/s)'); ylabel('Probability');
title('Wind Speed Distribution','FontWeight','bold');
sgtitle('Wind Disturbance Analysis','FontSize',14,'FontWeight','bold');

%% ===================== FIGURE 5: CONTROL PERFORMANCE =====================
figure('Name','Fig 5: Control','Position',[250 250 1200 600]);

subplot(2,2,1); hold on; grid on;
plot(T, vecnorm(d.control_output),'b-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('|u| (m/s)');
title('Total Control Effort','FontWeight','bold');

subplot(2,2,2); hold on; grid on;
plot(T, d.control_output(1,:),'r-','LineWidth',1.2,'DisplayName','Ux');
plot(T, d.control_output(2,:),'g-','LineWidth',1.2,'DisplayName','Uy');
plot(T, d.control_output(3,:),'b-','LineWidth',1.2,'DisplayName','Uz');
legend('Location','best');
xlabel('Time (s)'); ylabel('Control (m/s)');
title('Control Components','FontWeight','bold');

subplot(2,2,3);
plot(T, d.distance_to_platform,'k-','LineWidth',1.5); hold on; grid on;
xlabel('Time (s)'); ylabel('Distance (m)');
title('Distance to Platform','FontWeight','bold');

subplot(2,2,4);
plot(T, d.height_above_platform,'b-','LineWidth',1.5); hold on; grid on;
yline(0,'r--','LineWidth',2,'DisplayName','Platform level');
xlabel('Time (s)'); ylabel('Height (m)');
title('Height Above Platform','FontWeight','bold'); legend('Location','best');
sgtitle('Control System Performance','FontSize',14,'FontWeight','bold');

%% ===================== FIGURE 6: OBSTACLE AVOIDANCE =====================
figure('Name','Fig 6: Obstacle Avoidance','Position',[300 300 1200 500]);

subplot(1,2,1); hold on; grid on;
plot(T, vecnorm(d.avoidance_vel),'r-','LineWidth',1.5);
ob_t = T(d.obstacle_detected);
if ~isempty(ob_t)
    yl = ylim;
    for k = 1:length(ob_t)
        patch([ob_t(k)-0.1, ob_t(k)+0.1, ob_t(k)+0.1, ob_t(k)-0.1], ...
              [yl(1), yl(1), yl(2), yl(2)], ...
              'r','FaceAlpha',0.15,'EdgeColor','none');
    end
end
xlabel('Time (s)'); ylabel('Avoidance vel (m/s)');
title('Obstacle Avoidance Activation','FontWeight','bold');

subplot(1,2,2); hold on; grid on; axis equal;
plot(d.drone_pos(1,:),    d.drone_pos(2,:),   'b-','LineWidth',2,'DisplayName','Drone');
plot(d.platform_pos(1,:), d.platform_pos(2,:),'r--','LineWidth',2,'DisplayName','Platform');
th = linspace(0,2*pi,60);
fill(obstacle_pos(1)+obstacle_radius*cos(th), ...
     obstacle_pos(2)+obstacle_radius*sin(th), ...
     [1 0.5 0],'FaceAlpha',0.3,'EdgeColor','k','LineWidth',2,'DisplayName','Obstacle');
if any(d.obstacle_detected)
    di = find(d.obstacle_detected);
    plot(d.drone_pos(1,di), d.drone_pos(2,di), ...
        'r.','MarkerSize',8,'DisplayName','Avoidance on');
end
xlabel('North (m)'); ylabel('East (m)');
title('Top-Down: Avoidance Path','FontWeight','bold');
legend('Location','best');
sgtitle('Obstacle Detection and Avoidance','FontSize',14,'FontWeight','bold');

fprintf('All 6 figures generated.\n');
