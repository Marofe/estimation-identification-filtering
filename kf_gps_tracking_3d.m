%% ------------------------------------------------------------------------
%  SEL5917 - Bayesian Filtering and Smoothing
%  Example: Kalman Filter for 3D target tracking with a GPS sensor
%  Model:   Constant Velocity (CV) in 3D  +  position-only GPS measurements
%
%  State:   x = [px py pz vx vy vz]'     (position [m], velocity [m/s])
%  Sensor:  y = [px py pz]'  (GPS gives noisy x,y,z coordinates)
%
%  Trajectory: an ascending HELIX (climbing turn) that then levels off into
%  a horizontal CIRCLE at constant altitude.
%
%  GPS accuracy is anisotropic: the VERTICAL error (z) is larger than the
%  horizontal one, so the 2-sigma covariance regions are ELLIPSOIDS
%  (vertically elongated), drawn along the trajectory.
%
%  The single 3D animation shows, for each UPDATE step:
%    - true trajectory, noisy GPS measurements and the filtered estimate,
%    - the green 2-sigma POSTERIOR covariance ellipsoids along the path, and
%    - a simplified hexacopter marking the current estimate.
%
%  Running this file exports  images/kf_gps_tracking_3d.gif
%  Prof. Dr. Marcos Rogerio Fernandes
% -------------------------------------------------------------------------
clc; clear all; close all;
rng(7);                                  % reproducibility

%% -------------------- Simulation / model parameters ---------------------
dt   = 1.0;                              % sampling period [s]
N    = 48;                               % number of time steps
q    = 2.0;                              % process (acceleration) PSD [m^2/s^3]
sh   = 3.0;                              % GPS horizontal std (x,y) [m]
sv   = 10.0;                             % GPS vertical   std (z)  [m]  (larger!)

I3 = eye(3);  Z3 = zeros(3);

% Constant-velocity transition matrix  (block form)
A = [ I3, dt*I3;
      Z3,    I3 ];

% Continuous white-noise-acceleration process covariance
Q = q * [ dt^3/3*I3, dt^2/2*I3;
          dt^2/2*I3,    dt*I3   ];

% GPS measurement model: observes position only, anisotropic noise
H = [ I3, Z3 ];                          % 3 x 6
R = diag([sh^2, sh^2, sv^2]);            % measurement covariance

%% -------------------- Ground-truth trajectory ---------------------------
% Ascending helix (one climbing revolution) then a level circle.
t     = (0:N-1)*dt;
Rad   = 60;  Trev = 24;  w = 2*pi/Trev;  climb = 3;  nHel = 24;
zc    = climb*t;                         % climbing altitude ...
zTop  = climb*t(nHel);                   % ... until the top of the helix
zc(nHel:end) = zTop;                     % then level off (circle)
vz    = climb*ones(1,N);  vz(nHel:end) = 0;
Xtrue = [ Rad*cos(w*t);
          Rad*sin(w*t);
          zc;
         -Rad*w*sin(w*t);
          Rad*w*cos(w*t);
          vz ];

% GPS measurements (position + anisotropic Gaussian noise)
Y = H*Xtrue + diag([sh sh sv])*randn(3,N);

%% -------------------- Kalman filter initialization ----------------------
xhat = [Y(:,1); 0;0;0];                          % start at first GPS fix
P    = diag([sh^2 sh^2 sv^2 25 25 25]);          % initial covariance

Xest = zeros(6,N);   Xest(:,1) = xhat;
Pall = cell(1,N);    Pall{1}   = P;              % stored posterior covariances

%% -------------------- Figure / GIF setup -------------------------------
fig = figure('Color','w','Position',[80 80 1180 760],'Visible','off');
gifFile = fullfile('images','kf_gps_tracking_3d.gif');   % writes to slides/images
if ~exist('images','dir'); gifFile = 'kf_gps_tracking_3d.gif'; end
delay = 0.30;  first = true;  stride = 3;        % ellipsoid every 'stride' steps

% Precomputed axis limits (with room for the vertical ellipsoids)
lims = [min(Xtrue(1,:))-30 max(Xtrue(1,:))+30 ...
        min(Xtrue(2,:))-30 max(Xtrue(2,:))+30 ...
        min(Xtrue(3,:))-25 max(Xtrue(3,:))+25];

for k = 2:N
    % ---- PREDICTION (computed, not drawn) ----
    xpred = A*xhat;                      % x_{k|k-1} = A x_{k-1|k-1}
    Ppr   = A*P*A' + Q;                  % P_{k|k-1} = A P A' + Q
    % ---- UPDATE (drawn) ----
    yk = Y(:,k);                         % new GPS measurement
    S  = H*Ppr*H' + R;                   % innovation covariance
    K  = Ppr*H'/S;                       % Kalman gain
    xhat = xpred + K*(yk - H*xpred);     % x_{k|k}
    P    = (eye(6) - K*H)*Ppr;           % P_{k|k}
    Xest(:,k) = xhat;  Pall{k} = P;
    first = drawScene(fig,gifFile,first,delay, k, Xtrue,Y,Xest,Pall,xhat,P,lims,stride);
end

rmse = sqrt(mean(sum((Xest(1:3,:)-Xtrue(1:3,:)).^2,1)));
fprintf('Position RMSE = %.2f m   (GPS: sh=%.1f m, sv=%.1f m)\n',rmse,sh,sv);
fprintf('GIF written to %s\n', fullfile(pwd,gifFile));

%% ======================= helper functions ==============================
function first = drawScene(fig,gifFile,first,delay, k, Xtrue,Y,Xest,Pall,mu,Pcov,lims,stride)
    clf(fig); ax = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    green = [0.10 0.62 0.28];   blue = [0.00 0.45 0.85];

    % --- true trajectory + GPS measurements + filtered estimate ---
    h1 = plot3(ax,Xtrue(1,:),Xtrue(2,:),Xtrue(3,:),'-','Color',[.6 .6 .6],'LineWidth',1.3);
    h2 = plot3(ax,Y(1,1:k),Y(2,1:k),Y(3,1:k),'.','Color',blue,'MarkerSize',11);
    h3 = plot3(ax,Xest(1,1:k),Xest(2,1:k),Xest(3,1:k),'-','Color',green,'LineWidth',2.2);

    % --- faint (shadow) 2-sigma POSTERIOR ellipsoids ALONG the trajectory ---
    for j = 2:stride:k-1
        drawEllipsoid(ax, Xest(1:3,j), Pall{j}(1:3,1:3), [0.55 0.80 0.60], 0.12, 'none', '-', 'off');
    end
    h4 = drawEllipsoid(ax, mu(1:3), Pcov(1:3,1:3), [0.55 0.80 0.60], 0.18, 'none', '-', 'on');  % current (shadow)

    % --- hexacopter at the current estimate ---
    drawHexacopter(ax, mu, 7, green);

    axis(ax,lims); view(ax, 35 + k*1.3, 20); daspect(ax,[1 1 1]);
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); zlabel(ax,'z [m]');
    title(ax, sprintf('Kalman Filter  \\cdot  CV model + GPS (x,y,z)  \\cdot  step %d/%d', k, size(Xtrue,2)), ...
          'FontSize',13);
    text(ax, 0.02, 0.96, 'update step', 'Units','normalized', ...
         'Color',green,'FontWeight','bold','FontSize',14);
    lg = legend(ax, [h1 h2 h3 h4], {'true trajectory','GPS measurements','estimate x_{k|k}', ...
                     '2\sigma posterior ellipsoid'}, 'Location','northeast');
    set(lg,'Interpreter','tex','FontSize',9);

    % --- append frame to the GIF ---
    im = print(fig,'-RGBImage','-r96');
    [Aimg,map] = rgb2ind(im,256);
    if first
        imwrite(Aimg,map,gifFile,'gif','LoopCount',Inf,'DelayTime',delay); first = false;
    else
        imwrite(Aimg,map,gifFile,'gif','WriteMode','append','DelayTime',delay);
    end
end

function drawHexacopter(ax, mu, s, faceCol)
    % Simplified hexacopter (top view): central body + 6 arms with rotor disks.
    % The frame is oriented so one arm points along the heading (velocity).
    c  = mu(1:2);  z = mu(3);
    th0 = atan2(mu(5), mu(4));           % heading from vx, vy
    L  = s;  r = 0.42*s;                 % arm length and rotor radius
    tt = linspace(0,2*pi,24);
    ang = th0 + (0:5)*(pi/3);            % 6 arms, first aligned to heading
    for a = ang
        hub = c(:)' + L*[cos(a) sin(a)];
        plot3(ax,[c(1) hub(1)],[c(2) hub(2)],[z z],'-','Color',[.25 .25 .25], ...
              'LineWidth',1.6,'HandleVisibility','off');                       % arm
        rx = hub(1) + r*cos(tt);  ry = hub(2) + r*sin(tt);
        fill3(ax, rx, ry, z*ones(size(rx)), faceCol,'EdgeColor',[.2 .2 .2], ...
              'LineWidth',1.0,'FaceAlpha',0.45,'HandleVisibility','off');      % rotor disk
    end
    hubF = c(:)' + L*[cos(th0) sin(th0)];
    plot3(ax,[c(1) hubF(1)],[c(2) hubF(2)],[z z],'-','Color',faceCol*0.6, ...
          'LineWidth',2.6,'HandleVisibility','off');                          % front arm (heading)
    bx = c(1) + 0.55*r*cos(tt);  by = c(2) + 0.55*r*sin(tt);
    fill3(ax, bx, by, z*ones(size(bx)), faceCol,'EdgeColor','k', ...
          'LineWidth',1.0,'FaceAlpha',1,'HandleVisibility','off');            % central body
end

function h = drawEllipsoid(ax, mu, P, faceCol, faceAlpha, edgeCol, ls, handleVis)
    % 2-sigma ellipsoid of covariance P centered at mu
    [V,D] = eig((P+P')/2);  r = 2;                 % 2 sigma
    [xu,yu,zu] = sphere(16);
    pts = V*sqrt(max(D,0))*r*[xu(:)';yu(:)';zu(:)'] + mu(:);
    h = surf(ax, reshape(pts(1,:),size(xu)), reshape(pts(2,:),size(yu)), reshape(pts(3,:),size(zu)), ...
        'FaceColor',faceCol,'FaceAlpha',faceAlpha,'EdgeColor',edgeCol, ...
        'LineStyle',ls,'LineWidth',0.6,'HandleVisibility',handleVis);
end
