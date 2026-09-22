%% EKF — Target Tracking with Radar (hexacopter drone)
%  SEL5917 — Nonlinear Filtering (EKF)
%  Illustrates the Extended Kalman Filter applied to the model of slides 32-33:
%
%   State  x = [px; py; v; phi; Omega]      (Nearly Constant Turn model, 2D)
%   Dynamics f (slide 32):
%     px_{k+1}   = px + (v/Om)[ sin(phi+Om*T) - sin(phi) ]
%     py_{k+1}   = py + (v/Om)[ -cos(phi+Om*T) + cos(phi) ]
%     phi_{k+1}  = phi + Om*T
%     v_{k+1}    = v + w_k
%     Om_{k+1}   = Om + m_k
%   Radar measurement h (slide 33), polar:
%     y = [ r; theta ] = [ sqrt(px^2+py^2); atan2(py,px) ] + eps
%
%  EKF recursion (slide 30):
%     xhat_{k+1|k}   = f(xhat_{k|k})
%     P_{k+1|k}      = A_k P_{k|k} A_k' + Q
%     K              = P_{k+1|k} H' (R + H P_{k+1|k} H')^{-1}
%     xhat_{k+1|k+1} = xhat_{k+1|k} + K( y - h(xhat_{k+1|k}) )
%     P_{k+1|k+1}    = (I - K H) P_{k+1|k}
%     A_k = df/dx|xhat_{k|k},   H = dh/dx|xhat_{k+1|k}
%
%  Output: animated GIF  ../images/ekf_radar_drone.gif  and a poster PNG.
%  Author: generated for Prof. Marcos R. Fernandes.

clear; clc; close all; rng(7);

%% ---------------- parameters ----------------
T  = 1.0;                     % sample time [s]
N  = 70;                      % number of steps
sig_r     = 2.0;              % range noise std [m]
sig_theta = deg2rad(1.5);    % bearing noise std [rad]
R  = diag([sig_r^2, sig_theta^2]);

% process noise (enters mainly speed v and turn-rate Omega)
q_v  = 0.25^2;               % speed noise  [ (m/s)^2 ]
q_Om = deg2rad(1.2)^2;       % turn-rate noise [ (rad/s)^2 ]
Q    = diag([1e-4, 1e-4, q_v, 1e-6, q_Om]);

%% ---------------- ground-truth trajectory ----------------
% Compact, curvy flight around the radar (turn radius v/Omega ~ 90 m).
x = zeros(5,N);
x(:,1) = [ 95; -35; 6.0; deg2rad(120); deg2rad(3.8) ];  % [px py v phi Om]
for k = 1:N-1
    % scripted manoeuvre: change the commanded turn-rate a few times
    if     k==16, x(5,k) = deg2rad(-4.2);
    elseif k==34, x(5,k) = deg2rad( 3.6);
    elseif k==50, x(5,k) = deg2rad(-2.4);
    end
    w = [0;0; sqrt(q_v)*randn; 0; sqrt(q_Om)*randn];    % random speed/turn jitter
    x(:,k+1) = f_ct(x(:,k), T) + w;
end

%% ---------------- radar measurements ----------------
y = zeros(2,N);
for k = 1:N
    y(:,k) = h_radar(x(:,k)) + [sig_r; sig_theta].*randn(2,1);
end
% measurements shown in Cartesian (for plotting only)
mx = y(1,:).*cos(y(2,:));
my = y(1,:).*sin(y(2,:));

%% ---------------- EKF ----------------
xhat = zeros(5,N);
Phist = cell(1,N);
xhat(:,1) = x(:,1) + [8;-8;1.5;deg2rad(15);deg2rad(2)];  % imperfect initial guess
P = diag([15^2,15^2,3^2,deg2rad(25)^2,deg2rad(5)^2]);
Phist{1} = P;
I5 = eye(5);
for k = 1:N-1
    % --- predict ---
    A  = jac_f(xhat(:,k), T);
    xp = f_ct(xhat(:,k), T);
    Pp = A*P*A' + Q;
    % --- update with measurement y(:,k+1) ---
    H  = jac_h(xp);
    yp = h_radar(xp);
    innov = y(:,k+1) - yp;
    innov(2) = wrapToPi(innov(2));            % bearing residual wrapped to (-pi,pi]
    S  = H*Pp*H' + R;
    K  = Pp*H'/S;
    xhat(:,k+1) = xp + K*innov;
    xhat(4,k+1) = wrapToPi(xhat(4,k+1));
    P  = (I5 - K*H)*Pp;
    Phist{k+1} = P;
end

posRMSE = sqrt(mean( sum((xhat(1:2,:)-x(1:2,:)).^2,1) ));
fprintf('Position RMSE (EKF): %.2f m\n', posRMSE);
fprintf('Raw radar RMSE     : %.2f m\n', sqrt(mean((mx-x(1,:)).^2+(my-x(2,:)).^2)));

%% ---------------- animation -> GIF ----------------
here   = fileparts(mfilename('fullpath'));
gifOut = fullfile(here, '..', 'images', 'ekf_radar_drone.gif');
pngOut = fullfile(here, '..', 'images', 'ekf_radar_drone.png');

fig = figure('Color','w','Position',[100 100 760 720]);
ax  = axes(fig); hold(ax,'on'); axis(ax,'equal');
% square frame that includes the radar at the origin
allx=[x(1,:) mx 0]; ally=[x(2,:) my 0];
cx=(min(allx)+max(allx))/2; cy=(min(ally)+max(ally))/2;
half=max(max(allx)-min(allx), max(ally)-min(ally))/2 + 25;
lims=[cx-half cx+half cy-half cy+half];
hexScale = max(6, half*0.06);           % drone glyph size, scaled to the scene
ringStep = 30;                          % spacing of range rings [m]
axis(ax, lims); grid(ax,'on'); box(ax,'on');
xlabel(ax,'p_x  [m]'); ylabel(ax,'p_y  [m]');
cB=[0.18 0.42 0.85]; cR=[0.89 0.24 0.15]; cG=[0.07 0.60 0.42]; cK=[0.16 0.17 0.20];

for k = 1:N
    cla(ax);
    % range rings + radar station at origin
    for rr = ringStep:ringStep:(2*half)
        th=linspace(0,2*pi,120);
        plot(ax, rr*cos(th), rr*sin(th), '-', 'Color',[0.9 0.9 0.92]);
    end
    draw_radar(ax, cK, hexScale);
    % expanding radar wavefront (pulse)
    pr = mod(k*ringStep*0.5, 2*half);
    th=linspace(0,2*pi,160);
    plot(ax, pr*cos(th), pr*sin(th), '-', 'Color',[cG 0.5], 'LineWidth',1.2);
    % beam from radar to current target
    plot(ax, [0 x(1,k)], [0 x(2,k)], '-', 'Color',[cG 0.7], 'LineWidth',1.4);

    % measurements so far
    plot(ax, mx(1:k), my(1:k), '.', 'Color',[cR 0.5], 'MarkerSize',9);
    % true path
    plot(ax, x(1,1:k), x(2,1:k), '-', 'Color',cK, 'LineWidth',2.0);
    % EKF estimate path
    plot(ax, xhat(1,1:k), xhat(2,1:k), '-', 'Color',cB, 'LineWidth',2.2);
    % 2-sigma covariance ellipse at current estimate
    draw_ellipse(ax, xhat(1:2,k), Phist{k}(1:2,1:2), 2, cB);
    % hexacopter at the true position
    draw_hexacopter(ax, x(1:2,k), x(4,k), hexScale, cK, cB);
    % current measured point
    plot(ax, mx(k), my(k), 'o', 'Color',cR, 'MarkerFaceColor',[1 0.80 0.75], 'MarkerSize',7,'LineWidth',1.2);

    title(ax, sprintf('EKF radar tracking of a hexacopter   |   t = %4.1f s   |   pos. RMSE = %.1f m', (k-1)*T, posRMSE), ...
          'FontSize',13);
    % legend proxies
    hT=plot(ax,nan,nan,'-','Color',cK,'LineWidth',2);
    hM=plot(ax,nan,nan,'.','Color',cR,'MarkerSize',12);
    hE=plot(ax,nan,nan,'-','Color',cB,'LineWidth',2);
    legend(ax,[hT hM hE],{'true trajectory','radar measurements','EKF estimate'}, ...
           'Location','northoutside','Orientation','horizontal','FontSize',11,'Box','off');

    drawnow;
    frame = getframe(fig);
    [Aidx,cm] = rgb2ind(frame2im(frame),256);
    if k==1
        imwrite(Aidx,cm,gifOut,'gif','LoopCount',Inf,'DelayTime',0.10);
    else
        imwrite(Aidx,cm,gifOut,'gif','WriteMode','append','DelayTime',0.10);
    end
end
try, exportgraphics(fig, pngOut, 'Resolution',130); catch, saveas(fig,pngOut); end
fprintf('Saved GIF  -> %s\n', gifOut);
fprintf('Saved PNG  -> %s\n', pngOut);

%% ================= local functions =================
function a = wrapToPi(a)         % no toolbox needed
    a = mod(a + pi, 2*pi) - pi;
end

function xn = f_ct(x, T)
    px=x(1); py=x(2); v=x(3); ph=x(4); Om=x(5);
    if abs(Om) < 1e-6            % straight-line limit (Om -> 0)
        px2 = px + v*T*cos(ph);
        py2 = py + v*T*sin(ph);
    else
        px2 = px + (v/Om)*( sin(ph+Om*T) - sin(ph) );
        py2 = py + (v/Om)*( -cos(ph+Om*T) + cos(ph) );
    end
    xn = [px2; py2; v; ph+Om*T; Om];
end

function A = jac_f(x, T)        % numerical Jacobian df/dx (central differences)
    n=numel(x); A=zeros(n); h=1e-6;
    for j=1:n
        dp=zeros(n,1); dp(j)=h;
        A(:,j) = (f_ct(x+dp,T) - f_ct(x-dp,T))/(2*h);
    end
end

function yv = h_radar(x)        % polar measurement
    yv = [ hypot(x(1),x(2)); atan2(x(2),x(1)) ];
end

function H = jac_h(x)           % analytic dh/dx
    px=x(1); py=x(2); r2=px^2+py^2; r=sqrt(r2);
    H = [ px/r ,  py/r , 0,0,0 ;
         -py/r2,  px/r2, 0,0,0 ];
end

function draw_radar(ax, c, s)
    % small antenna/dish glyph at the origin, scaled by s
    plot(ax,0,0,'^','MarkerSize',12,'MarkerFaceColor',c,'MarkerEdgeColor',c);
    plot(ax,[0 0],[0 s],'-','Color',c,'LineWidth',1.5);
    th=linspace(-pi/3,pi/3,20);
    plot(ax, 0.85*s*sin(th), s+0.42*s*(cos(th)-1), '-','Color',c,'LineWidth',1.5);
    text(ax, 0.5*s, -s, 'RADAR','Color',c,'FontSize',9,'FontWeight','bold');
end

function draw_ellipse(ax, mu, C, ns, col)
    C = (C+C')/2;
    [V,D] = eig(C); D=max(D,0);
    th=linspace(0,2*pi,80);
    e = V*sqrt(D)*[cos(th); sin(th)]*ns;
    plot(ax, mu(1)+e(1,:), mu(2)+e(2,:), '-', 'Color',[col 0.9], 'LineWidth',1.2);
end

function draw_hexacopter(ax, c, heading, s, cArm, cRotor)
    % top-view hexacopter: hub + 6 arms + 6 rotor discs
    c=c(:);
    for a = 0:60:300
        ang = deg2rad(a) + heading;
        d = c + s*[cos(ang); sin(ang)];
        plot(ax,[c(1) d(1)],[c(2) d(2)],'-','Color',cArm,'LineWidth',2);   % arm
        th=linspace(0,2*pi,24); rr=s*0.42;
        patch(ax, d(1)+rr*cos(th), d(2)+rr*sin(th), cRotor, ...
              'FaceAlpha',0.35,'EdgeColor',cRotor,'LineWidth',1.2);         % rotor
    end
    th=linspace(0,2*pi,24);
    patch(ax, c(1)+s*0.35*cos(th), c(2)+s*0.35*sin(th), cArm, 'EdgeColor',cArm); % hub
end
