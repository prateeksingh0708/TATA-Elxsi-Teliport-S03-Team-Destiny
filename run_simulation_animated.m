% run_simulation_animated.m
% UAV Landing Simulation with Phase-Coloured Track Animation
% Run with: >> run_simulation_animated

clear all; close all; clc;

%% ===================== PARAMETERS =====================
dt               = 0.05;
T_max            = 40;
time             = 0:dt:T_max;
platform_init    = [15;0;0];
platform_vel_vec = [2.5;0.3;0];
platform_dims    = [8;2.5;1];
steady_wind      = [1.5;1.0;0];
gust_amp         = 2.0;
turb_level       = 0.3;
obstacles        = [25,1,-3,1.5; 40,-1,-2.5,2.0];
Kp = [1.8;1.8;2.2];
Ki = [0.15;0.15;0.25];
Kd = [0.9;0.9;1.2];

%% ===================== OBJECTS =====================
drone             = DroneSimulator([0;-10;-8], dt);
platform          = MovingPlatform(platform_init, platform_vel_vec, platform_dims);
windModel         = CrosswindModel(steady_wind, gust_amp, turb_level);
pidCtrl           = PID_Controller(Kp, Ki, Kd, dt);
trajPlanner       = TrajectoryPlanner(platform);
obsAvoid          = ObstacleAvoidance();
for k = 1:size(obstacles,1)
    obsAvoid.addObstacle(obstacles(k,1:3)', obstacles(k,4));
end
fprintf('Controllers ready\n\n');

%% ===================== PHASE COLOURS =====================
COL.approach = [0.20, 0.45, 0.90];
COL.sync     = [0.95, 0.55, 0.05];
COL.descend  = [0.90, 0.18, 0.15];
COL.landed   = [0.10, 0.72, 0.22];
COL.default  = [0.70, 0.70, 0.70];

%% ===================== FIGURE =====================
fig = figure('Name','UAV Landing Animation','Position',[50 50 1500 900]);
ax  = axes('Parent',fig);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax,45,25);
axis(ax,[-5 65 -16 12 -11 11]);
ax.Color     = [0.07 0.07 0.12];
fig.Color    = [0.07 0.07 0.12];
ax.GridColor = [0.35 0.35 0.35];
ax.GridAlpha = 0.5;
ax.XColor = [0.85 0.85 0.85];
ax.YColor = [0.85 0.85 0.85];
ax.ZColor = [0.85 0.85 0.85];
xlabel(ax,'North (m)','FontSize',12,'Color',[0.9 0.9 0.9]);
ylabel(ax,'East (m)' ,'FontSize',12,'Color',[0.9 0.9 0.9]);
zlabel(ax,'Up (m)'   ,'FontSize',12,'Color',[0.9 0.9 0.9]);
title(ax,'UAV Landing - Phase Track','FontSize',14,'FontWeight','bold','Color',[1 1 1]);

%% Draw obstacles
for k = 1:size(obstacles,1)
    [Xc,Yc,Zc] = cylinder(obstacles(k,4), 40);
    Zc = Zc*22 - 11;
    surf(ax, obstacles(k,1)+Xc, obstacles(k,2)+Yc, -obstacles(k,3)+Zc, ...
        'FaceColor',[0.75,0.28,0.08],'FaceAlpha',0.55,'EdgeColor',[0.4,0.15,0.02]);
    th = linspace(0,2*pi,80);
    plot3(ax, obstacles(k,1)+obstacles(k,4)*1.2*cos(th), ...
              obstacles(k,2)+obstacles(k,4)*1.2*sin(th), ...
              -obstacles(k,3)*ones(1,80), ...
        'Color',[1 0.4 0.1],'LineStyle',':','LineWidth',1.3);
end

%% Legend proxies
plot3(ax,nan,nan,nan,'-','Color',COL.approach,'LineWidth',2.5,'DisplayName','Approach');
plot3(ax,nan,nan,nan,'-','Color',COL.sync    ,'LineWidth',2.5,'DisplayName','Sync');
plot3(ax,nan,nan,nan,'-','Color',COL.descend ,'LineWidth',2.5,'DisplayName','Descend');
plot3(ax,nan,nan,nan,'-','Color',COL.landed  ,'LineWidth',2.5,'DisplayName','Landed');
plot3(ax,nan,nan,nan,'r--','LineWidth',1.8,'DisplayName','Platform trail');
leg = legend(ax,'Location','northwest','TextColor',[0.9 0.9 0.9], ...
    'Color',[0.12 0.12 0.18],'EdgeColor',[0.4 0.4 0.5]);
leg.FontSize = 9;

%% Track line handles (one per phase, grows each step)
trk.approach = plot3(ax,nan,nan,nan,'-','Color',COL.approach,'LineWidth',2.5);
trk.sync     = plot3(ax,nan,nan,nan,'-','Color',COL.sync    ,'LineWidth',2.5);
trk.descend  = plot3(ax,nan,nan,nan,'-','Color',COL.descend ,'LineWidth',2.5);
trk.landed   = plot3(ax,nan,nan,nan,'-','Color',COL.landed  ,'LineWidth',2.5);
plat_trail   = plot3(ax,nan,nan,nan,'r--','LineWidth',1.8);

%% Start marker
s0 = drone.getState();
plot3(ax,s0.position(1),s0.position(2),-s0.position(3), ...
    'g^','MarkerSize',13,'MarkerFaceColor','g','LineWidth',2);

%% Quadcopter handle and info box
quad_h   = draw_quadcopter(s0.position,[0,0,0],1.5);
plt_patch = [];
info_box  = text(ax,2,-13,9,'','FontSize',10, ...
    'BackgroundColor',[0.10 0.10 0.15],'EdgeColor',[0.5 0.5 0.6], ...
    'Color',[1 1 1],'LineWidth',1.5,'VerticalAlignment','top');

%% Position buffers per phase
buf.approach = zeros(3,0);
buf.sync     = zeros(3,0);
buf.descend  = zeros(3,0);
buf.landed   = zeros(3,0);

%% Log arrays
N          = length(time);
log_pos    = zeros(3,N);
log_plat   = zeros(3,N);
log_vel    = zeros(3,N);
log_pvel   = zeros(3,N);
log_wind   = zeros(3,N);
log_ctrl   = zeros(3,N);
log_avoid  = zeros(3,N);
log_err    = zeros(3,N);
log_dist   = zeros(1,N);
log_height = zeros(1,N);
log_obs    = false(1,N);
log_phase  = cell(1,N);

landed       = false;
landing_time = inf;
prev_phase   = 'approach';
last_i       = 1;

fprintf('Running simulation...\n');

%% ===================== MAIN LOOP =====================
for i = 1:N
    t = time(i);

    st          = drone.getState();
    drone_pos   = st.position;
    drone_vel   = st.velocity;
    plat_pos    = platform.getPosition(t);
    plat_vel    = platform.getVelocity(t);

    [tgt_pos, tgt_vel, phase] = trajPlanner.getTarget(drone_pos, drone_vel, t);
    pos_err     = tgt_pos - drone_pos;
    ctrl_vel    = pidCtrl.compute(pos_err) + tgt_vel;

    [avoid_vel, obs_det, ~] = obsAvoid.computeAvoidance(drone_pos);
    ctrl_vel    = ctrl_vel + avoid_vel;

    wind_vel    = windModel.getWind(t);
    wind_force  = windModel.getWindForce(t, 0.08, drone_vel);
    ctrl_vel    = ctrl_vel - 0.5*wind_vel;

    if norm(ctrl_vel) > 6
        ctrl_vel = 6 * ctrl_vel / norm(ctrl_vel);
    end

    if ~strcmp(phase,'landed')
        drone.moveByVelocity(ctrl_vel(1),ctrl_vel(2),ctrl_vel(3),dt);
    else
        drone.hover();
        if ~landed
            landed = true;
            landing_time = t;
            fprintf('\nLanded at %.1f s\n', t);
        end
    end
    drone.update(wind_force);

    %% Store logs
    log_pos(:,i)    = drone_pos;
    log_plat(:,i)   = plat_pos;
    log_vel(:,i)    = drone_vel;
    log_pvel(:,i)   = plat_vel;
    log_wind(:,i)   = wind_vel;
    log_ctrl(:,i)   = ctrl_vel;
    log_avoid(:,i)  = avoid_vel;
    log_err(:,i)    = pos_err;
    log_dist(i)     = norm(drone_pos - plat_pos);
    log_height(i)   = -drone_pos(3) - (-plat_pos(3));
    log_obs(i)      = obs_det;
    log_phase{i}    = phase;

    %% Append to phase buffer
    switch phase
        case 'approach', buf.approach = [buf.approach, drone_pos];
        case 'sync',     buf.sync     = [buf.sync,     drone_pos];
        case 'descend',  buf.descend  = [buf.descend,  drone_pos];
        case 'landed',   buf.landed   = [buf.landed,   drone_pos];
    end

    %% Phase transition dot
    if ~strcmp(phase, prev_phase)
        if isfield(COL, prev_phase)
            col_p = COL.(prev_phase);
        else
            col_p = COL.default;
        end
        plot3(ax,drone_pos(1),drone_pos(2),-drone_pos(3),'o', ...
            'MarkerSize',9,'MarkerFaceColor',col_p,'MarkerEdgeColor','w','LineWidth',1.5);
        fprintf('  [t=%.1fs] -> %s\n', t, upper(phase));
        prev_phase = phase;
    end

    %% Animate every 2nd step
    if mod(i,2) == 0

        yaw = 0;
        if norm(drone_vel(1:2)) > 0.1
            yaw = atan2(drone_vel(2),drone_vel(1));
        end
        update_quadcopter(quad_h, drone_pos, [0,0,yaw]);

        % Phase tracks
        phases_list = {'approach','sync','descend','landed'};
        for p = 1:4
            pn = phases_list{p};
            pts = buf.(pn);
            if size(pts,2) >= 2
                set(trk.(pn),'XData',pts(1,:),'YData',pts(2,:),'ZData',-pts(3,:));
            end
        end

        % Platform trail
        set(plat_trail,'XData',log_plat(1,1:i),'YData',log_plat(2,1:i),'ZData',-log_plat(3,1:i));

        % Platform patch
        if ~isempty(plt_patch), delete(plt_patch); end
        Lp = platform_dims(1); Wp = platform_dims(2);
        px = [plat_pos(1)+Lp/2, plat_pos(1)+Lp/2, plat_pos(1)-Lp/2, plat_pos(1)-Lp/2];
        py = [plat_pos(2)+Wp/2, plat_pos(2)-Wp/2, plat_pos(2)-Wp/2, plat_pos(2)+Wp/2];
        pz = [-plat_pos(3), -plat_pos(3), -plat_pos(3), -plat_pos(3)];
        plt_patch = patch(ax,px,py,pz,[0.15,0.75,0.15],'FaceAlpha',0.65,'EdgeColor','w','LineWidth',2);

        % Info box
        if isfield(COL,phase)
            col_c = COL.(phase);
        else
            col_c = COL.default;
        end
        info_str = sprintf('Time:  %.1f s\nPhase: %s\nAlt:   %.1f m\nSpeed: %.2f m/s\nWind:  %.1f m/s', ...
            t, upper(phase), -drone_pos(3), norm(drone_vel), norm(wind_vel));
        set(info_box,'String',info_str,'Color',col_c);

        % Camera
        if mod(i,10) == 0
            cam = drone_pos + [-14;-7;7];
            campos(ax,[cam(1),cam(2),-cam(3)]);
            camtarget(ax,[drone_pos(1),drone_pos(2),-drone_pos(3)]);
        end

        drawnow limitrate;
    end

    last_i = i;
    if landed && t > (landing_time+3), break; end
end

%% Landing star
fi = find(strcmp(log_phase(1:last_i),'landed'),1,'first');
if ~isempty(fi)
    fp = log_pos(:,fi);
    plot3(ax,fp(1),fp(2),-fp(3),'p','MarkerSize',18, ...
        'MarkerFaceColor',[0.1 0.9 0.3],'MarkerEdgeColor','w','LineWidth',2);
end

%% ===================== SAVE LOG =====================
simlog.time             = time(1:last_i);
simlog.drone_pos        = log_pos(:,1:last_i);
simlog.platform_pos     = log_plat(:,1:last_i);
simlog.drone_vel        = log_vel(:,1:last_i);
simlog.platform_vel     = log_pvel(:,1:last_i);
simlog.wind_vel         = log_wind(:,1:last_i);
simlog.control_output   = log_ctrl(:,1:last_i);
simlog.avoidance_vel    = log_avoid(:,1:last_i);
simlog.pos_error        = log_err(:,1:last_i);
simlog.distance_to_platform  = log_dist(1:last_i);
simlog.height_above_platform = log_height(1:last_i);
simlog.obstacle_detected     = log_obs(1:last_i);
simlog.phase            = log_phase(1:last_i);

homeDir = getenv('HOME');
if isempty(homeDir), homeDir = getenv('USERPROFILE'); end
save_dir = fullfile(homeDir,'Desktop','UAV_Landing_Project','Data','simulation_logs');
if ~exist(save_dir,'dir'), mkdir(save_dir); end
fname = fullfile(save_dir, ['sim_' datestr(now,'yyyymmdd_HHMMSS') '.mat']);
save(fname,'simlog');
fprintf('Log saved: %s\n', fname);
fprintf('Landing time: %.1f s\n', landing_time);
fprintf('Animation complete!\n');
