%% ------------------------------------------------------------------------
%  SEL5917 - Bayesian Filtering and Smoothing
%  Example: RTS Smoother for 3D target tracking with a GPS sensor
%  Model:   Constant Velocity (CV) in 3D  +  position-only GPS measurements
%  (same model, trajectory and measurements as kf_gps_tracking_3d.m)
%
%  Runs the Kalman Filter (forward) and the Rauch-Tung-Striebel smoother
%  (backward) and exports:
%     images/rts_3d_compare.gif        (animated: forward filter, backward smoother)
%     images/rts_pos_compare.png       (positions:  x, y, z)
%     images/rts_vel_compare.png       (velocities: vx, vy, vz)
%
%  Prof. Dr. Marcos Rogerio Fernandes
% -------------------------------------------------------------------------
clc; clear all; close all;
rng(7);                                  % same seed as the filter example

%% -------------------- Model / trajectory (identical) --------------------
dt = 1.0;  N = 48;  q = 2.0;  sh = 3.0;  sv = 10.0;
I3 = eye(3);  Z3 = zeros(3);
A = [ I3, dt*I3;  Z3, I3 ];
Q = q * [ dt^3/3*I3, dt^2/2*I3;  dt^2/2*I3, dt*I3 ];
H = [ I3, Z3 ];
R = diag([sh^2, sh^2, sv^2]);

t   = (0:N-1)*dt;
Rad = 60;  Trev = 24;  w = 2*pi/Trev;  climb = 3;  nHel = 24;
zc  = climb*t;  zc(nHel:end) = climb*t(nHel);
vz  = climb*ones(1,N);  vz(nHel:end) = 0;
Xtrue = [ Rad*cos(w*t); Rad*sin(w*t); zc; ...
         -Rad*w*sin(w*t); Rad*w*cos(w*t); vz ];
Y = H*Xtrue + diag([sh sh sv])*randn(3,N);

%% -------------------- Kalman Filter (forward pass) ----------------------
xhat = [Y(:,1); 0;0;0];
P    = diag([sh^2 sh^2 sv^2 25 25 25]);
Xf = zeros(6,N);  Pf = cell(1,N);
Xf(:,1) = xhat;   Pf{1} = P;
for k = 2:N
    xpred = A*xhat;             Ppr = A*P*A' + Q;
    S = H*Ppr*H' + R;           K   = Ppr*H'/S;
    xhat = xpred + K*(Y(:,k) - H*xpred);
    P    = (eye(6) - K*H)*Ppr;
    Xf(:,k) = xhat;             Pf{k} = P;
end

%% -------------------- RTS Smoother (backward pass) ----------------------
Xs = zeros(6,N);  Ps = cell(1,N);
Xs(:,N) = Xf(:,N);  Ps{N} = Pf{N};
for k = N-1:-1:1
    Pminus = A*Pf{k}*A' + Q;                 % P_{k+1}^-
    G      = Pf{k}*A'/Pminus;                % smoother gain G_k
    Xs(:,k) = Xf(:,k) + G*(Xs(:,k+1) - A*Xf(:,k));
    Ps{k}   = Pf{k} + G*(Ps{k+1} - Pminus)*G';
end

%% -------------------- Error metrics ------------------------------------
rmsePos = @(Xe) sqrt(mean(sum((Xe(1:3,:)-Xtrue(1:3,:)).^2,1)));
rmseVel = @(Xe) sqrt(mean(sum((Xe(4:6,:)-Xtrue(4:6,:)).^2,1)));
fprintf('Position RMSE:  filter = %.2f m,  smoother = %.2f m\n', rmsePos(Xf), rmsePos(Xs));
fprintf('Velocity RMSE:  filter = %.2f m/s, smoother = %.2f m/s\n', rmseVel(Xf), rmseVel(Xs));

gray=[0.20 0.20 0.20]; green=[0 0.4470 0.7410]; purple=[0.8500 0.3250 0.0980]; blue=[0.60 0.60 0.60];

%% ================= Figure 1: animated 3D comparison (GIF) ===============
fig = figure('Color','w','Position',[80 80 1180 760],'Visible','off');
gifFile = fullfile('images','rts_3d_compare.gif');
if ~exist('images','dir'); gifFile = 'rts_3d_compare.gif'; end
delay = 0.16;  first = true;
lims = [min(Xtrue(1,:))-30 max(Xtrue(1,:))+30 ...
        min(Xtrue(2,:))-30 max(Xtrue(2,:))+30 ...
        min(Xtrue(3,:))-25 max(Xtrue(3,:))+25];
step = 0;

% ---- Phase 1: forward Kalman filter ----
for k = 1:N
    step = step + 1;
    first = frame3d(fig,gifFile,first,delay, step, lims, ...
        Xtrue, Y, k, Xf(:,1:k), [], Xf(:,k), green, ...
        'Kalman Filter  (forward:  y_{0:k})', green);
end
% ---- Phase 2: backward RTS smoother ----
for k = N:-1:1
    step = step + 1;
    first = frame3d(fig,gifFile,first,delay, step, lims, ...
        Xtrue, Y, N, Xf, Xs(:,k:N), Xs(:,k), purple, ...
        'RTS Smoother  (backward:  y_{0:T})', purple);
end
% ---- hold the final frame a bit longer ----
for r = 1:6
    first = frame3d(fig,gifFile,first,delay, step, lims, ...
        Xtrue, Y, N, Xf, Xs, Xs(:,1), purple, ...
        'RTS Smoother  (backward:  y_{0:T})', purple);
end

%% ================= Figure 2: positions x, y, z =========================
f2 = figure('Color','w','Position',[60 80 1500 470],'Visible','off');
tl = tiledlayout(f2,1,3,'TileSpacing','compact','Padding','compact');
labs = {'x [m]','y [m]','z [m]'};
for c = 1:3
    nexttile(tl); hold on; grid on; box on;
    plot(t,Xtrue(c,:),'-','Color',gray,'LineWidth',2.4);
    plot(t,Xf(c,:),'--','Color',green,'LineWidth',1.7);
    plot(t,Xs(c,:),'-','Color',purple,'LineWidth',1.9);
    xlabel('time step k'); ylabel(labs{c}); title(labs{c}); xlim([t(1) t(end)]);
end
lg = legend({'true','filter','smoother'},'Orientation','horizontal'); lg.Layout.Tile='north';
title(tl,'Filter vs RTS Smoother  \cdot  positions','FontSize',15,'FontWeight','bold');
exportgraphics(f2, fullfile('images','rts_pos_compare.png'), 'Resolution',150);

%% ================= Figure 3: velocities vx, vy, vz =====================
f3 = figure('Color','w','Position',[60 80 1500 470],'Visible','off');
tl3 = tiledlayout(f3,1,3,'TileSpacing','compact','Padding','compact');
labv = {'v_x [m/s]','v_y [m/s]','v_z [m/s]'};
for c = 4:6
    nexttile(tl3); hold on; grid on; box on;
    plot(t,Xtrue(c,:),'-','Color',gray,'LineWidth',2.4);
    plot(t,Xf(c,:),'--','Color',green,'LineWidth',1.7);
    plot(t,Xs(c,:),'-','Color',purple,'LineWidth',1.9);
    xlabel('time step k'); ylabel(labv{c-3}); title(labv{c-3}); xlim([t(1) t(end)]);
end
lg3 = legend({'true','filter','smoother'},'Orientation','horizontal'); lg3.Layout.Tile='north';
title(tl3,'Filter vs RTS Smoother  \cdot  velocities','FontSize',15,'FontWeight','bold');
exportgraphics(f3, fullfile('images','rts_vel_compare.png'), 'Resolution',150);

%% ================= Figure 4: error histograms (filter vs smoother) =====
Ef = Xf - Xtrue;  Es = Xs - Xtrue;                 % estimation errors
f4 = figure('Color','w','Position',[60 60 1440 780],'Visible','off');
tl4 = tiledlayout(f4,2,3,'TileSpacing','compact','Padding','compact');
alln = {'x [m]','y [m]','z [m]','v_x [m/s]','v_y [m/s]','v_z [m/s]'};
for c = 1:6
    nexttile(tl4); hold on; box on; grid on;
    lo = min([Ef(c,:) Es(c,:)]);  hi = max([Ef(c,:) Es(c,:)]);
    edges = linspace(lo,hi,16);
    cF = [0 0.4470 0.7410];  cS = [0.8500 0.3250 0.0980];   % default MATLAB colors
    histogram(Ef(c,:),edges,'FaceColor',cF,'FaceAlpha',0.55,'EdgeColor','none');
    histogram(Es(c,:),edges,'FaceColor',cS,'FaceAlpha',0.55,'EdgeColor','none');
    xline(0,'k--','LineWidth',1.0);
    xlabel(['error   ' alln{c}]);  ylabel('count');  title(alln{c});
end
lg4 = legend({'filter','smoother'},'Orientation','horizontal'); lg4.Layout.Tile='north';
title(tl4,'Estimation-error histograms  \cdot  filter vs smoother', ...
      'FontSize',15,'FontWeight','bold');
exportgraphics(f4, fullfile('images','rts_error_hist.png'), 'Resolution',140);

%% ---- per-variable RMSE table (printed to the command window) ----------
fprintf('\nRMSE per variable  (filter | smoother | improvement):\n');
names = {'x','y','z','vx','vy','vz'};  units = {'m','m','m','m/s','m/s','m/s'};
for c = 1:6
    rf = sqrt(mean(Ef(c,:).^2));  rs = sqrt(mean(Es(c,:).^2));
    fprintf('  %-3s : %7.3f | %7.3f %-4s (%.0f%%)\n', names{c}, rf, rs, units{c}, 100*(1-rs/rf));
end

fprintf('Saved rts_3d_compare.gif, rts_pos_compare.png, rts_vel_compare.png, rts_error_hist.png\n');

%% ======================= helper functions ==============================
function first = frame3d(fig,gifFile,first,delay, step, lims, Xtrue,Y,kGps, ...
                         Xfil, Xsm, mu, muCol, phaseTxt, phaseCol)
    clf(fig); ax = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    gray=[0.20 0.20 0.20]; green=[0 0.4470 0.7410]; purple=[0.8500 0.3250 0.0980]; blue=[0.60 0.60 0.60];
    plot3(ax,Xtrue(1,:),Xtrue(2,:),Xtrue(3,:),'-','Color',gray,'LineWidth',1.6);
    plot3(ax,Y(1,1:kGps),Y(2,1:kGps),Y(3,1:kGps),'.','Color',blue,'MarkerSize',9);
    if ~isempty(Xfil)
        plot3(ax,Xfil(1,:),Xfil(2,:),Xfil(3,:),'-','Color',green,'LineWidth',2.0);
    end
    if ~isempty(Xsm)
        plot3(ax,Xsm(1,:),Xsm(2,:),Xsm(3,:),'-','Color',purple,'LineWidth',2.2);
    end
    drawHexacopter(ax, mu, 7, muCol);
    axis(ax,lims); view(ax, 35 + step*0.9, 20); daspect(ax,[1 1 1]);
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); zlabel(ax,'z [m]');
    title(ax,'Kalman Filter vs RTS Smoother  \cdot  CV + GPS (3D)','FontSize',13);
    text(ax,0.02,0.96,phaseTxt,'Units','normalized','Color',phaseCol, ...
         'FontWeight','bold','FontSize',13,'Interpreter','tex');
    lg = legend(ax,{'true trajectory','GPS measurements','filter','smoother'},'Location','northeast');
    set(lg,'FontSize',9);
    im = print(fig,'-RGBImage','-r96');
    [Aimg,map] = rgb2ind(im,256);
    if first
        imwrite(Aimg,map,gifFile,'gif','LoopCount',Inf,'DelayTime',delay); first=false;
    else
        imwrite(Aimg,map,gifFile,'gif','WriteMode','append','DelayTime',delay);
    end
end

function drawHexacopter(ax, mu, s, faceCol)
    c  = mu(1:2);  z = mu(3);
    th0 = atan2(mu(5), mu(4));
    L  = s;  r = 0.42*s;
    tt = linspace(0,2*pi,24);
    for a = th0 + (0:5)*(pi/3)
        hub = c(:)' + L*[cos(a) sin(a)];
        plot3(ax,[c(1) hub(1)],[c(2) hub(2)],[z z],'-','Color',[.25 .25 .25], ...
              'LineWidth',1.6,'HandleVisibility','off');
        rx = hub(1) + r*cos(tt);  ry = hub(2) + r*sin(tt);
        fill3(ax, rx, ry, z*ones(size(rx)), faceCol,'EdgeColor',[.2 .2 .2], ...
              'LineWidth',1.0,'FaceAlpha',0.45,'HandleVisibility','off');
    end
    hubF = c(:)' + L*[cos(th0) sin(th0)];
    plot3(ax,[c(1) hubF(1)],[c(2) hubF(2)],[z z],'-','Color',faceCol*0.6, ...
          'LineWidth',2.6,'HandleVisibility','off');
    bx = c(1) + 0.55*r*cos(tt);  by = c(2) + 0.55*r*sin(tt);
    fill3(ax, bx, by, z*ones(size(bx)), faceCol,'EdgeColor','k', ...
          'LineWidth',1.0,'FaceAlpha',1,'HandleVisibility','off');
end
