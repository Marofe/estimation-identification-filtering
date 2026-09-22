%% Polar -> Cartesian : Monte-Carlo truth vs first-order (EKF) linearization
%  SEL5917 — Nonlinear Filtering
%  A Gaussian in polar coordinates (r, theta) is pushed through the nonlinear map
%       g(r,theta) = [ r*cos(theta) ; r*sin(theta) ]
%  We compare:
%    (MC)   the true transformed distribution: samples + sample mean/cov ellipse
%    (LIN)  the first-order Taylor (EKF) approximation:
%              mean_lin = g(mu),   cov_lin = G * Sigma * G',   G = dg/d[r,theta]|mu
%  Sweeping the bearing std sigma_theta shows how badly the linearization fails
%  as the nonlinearity (angular spread) grows.
%
%  Output:  ../images/polar2cart_linearization.gif  and  .png
%  Author: generated for Prof. Marcos R. Fernandes.

clear; clc; close all; rng(3);

%% input Gaussian in polar coordinates
r0   = 10;                 % mean range  [m]
th0  = deg2rad(70);        % mean bearing
sr   = 0.35;               % range std   [m]
Nmc  = 4000;               % Monte-Carlo samples

% fixed standard-normal draws so the cloud morphs smoothly across frames
z1 = randn(Nmc,1);  z2 = randn(Nmc,1);

% bearing std sweep (deg): grow, then hold on the worst case
sth_deg = [linspace(3,34,34), 34*ones(1,8)];

here   = fileparts(mfilename('fullpath'));
gifOut = fullfile(here,'..','images','polar2cart_linearization.gif');
pngOut = fullfile(here,'..','images','polar2cart_linearization.png');

fig = figure('Color','w','Position',[80 80 1150 560]);
cMC=[0.09 0.60 0.40]; cLIN=[0.89 0.20 0.13]; cPT=[0.16 0.17 0.20];

for fidx = 1:numel(sth_deg)
    sth = deg2rad(sth_deg(fidx));
    Sigma = diag([sr^2, sth^2]);

    % --- Monte-Carlo truth ---
    r  = r0 + sr*z1;
    th = th0 + sth*z2;
    X  = r.*cos(th);   Y = r.*sin(th);
    P  = [X Y];
    muMC  = mean(P,1)';
    covMC = cov(P);

    % --- first-order (EKF) linearization ---
    muL = [r0*cos(th0); r0*sin(th0)];
    G   = [cos(th0), -r0*sin(th0);
           sin(th0),  r0*cos(th0)];
    covL = G*Sigma*G';

    meanErr = norm(muMC - muL);

    clf(fig);
    % ---------- left: input Gaussian in polar space ----------
    ax1 = subplot(1,2,1); hold(ax1,'on'); box(ax1,'on'); grid(ax1,'on');
    scatter(ax1, rad2deg(th), r, 6, [0.6 0.6 0.62],'filled','MarkerFaceAlpha',0.25);
    draw_ellipse(ax1, [rad2deg(th0); r0], diag([rad2deg(sth)^2, sr^2]), 2, cMC);
    plot(ax1, rad2deg(th0), r0, 'o','Color',cMC,'MarkerFaceColor',cMC,'MarkerSize',7);
    xlabel(ax1,'bearing  \theta  [deg]'); ylabel(ax1,'range  r  [m]');
    title(ax1,'Input:  Gaussian in polar space','FontSize',12);
    xlim(ax1,[th0*180/pi-105, th0*180/pi+105]); ylim(ax1,[r0-4*sr, r0+4*sr]);

    % ---------- right: Cartesian, MC truth vs linearization ----------
    ax2 = subplot(1,2,2); hold(ax2,'on'); box(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal');
    scatter(ax2, X, Y, 6, [0.62 0.62 0.66],'filled','MarkerFaceAlpha',0.20);
    hT = draw_ellipse(ax2, muMC, covMC, 2, cMC);    % true (MC)
    hL = draw_ellipse(ax2, muL,  covL,  2, cLIN);   % linearized (EKF)
    plot(ax2, muMC(1),muMC(2),'o','Color',cMC,'MarkerFaceColor',cMC,'MarkerSize',7);
    plot(ax2, muL(1), muL(2), 's','Color',cLIN,'MarkerFaceColor',cLIN,'MarkerSize',8);
    % radar at origin + ray to the mean point
    plot(ax2,0,0,'^','Color',cPT,'MarkerFaceColor',cPT,'MarkerSize',9);
    plot(ax2,[0 muL(1)],[0 muL(2)],'-','Color',[cPT 0.4]);
    xlabel(ax2,'p_x  [m]'); ylabel(ax2,'p_y  [m]');
    xlim(ax2,[-7 10]); ylim(ax2,[1 12]);
    legend(ax2,[hT hL],{'Monte-Carlo (true)','1^{st}-order Taylor / EKF'}, ...
           'Location','southwest','FontSize',10,'Box','off');
    title(ax2, sprintf('Cartesian:  \\sigma_\\theta = %2.0f^\\circ    |    mean error = %.2f m', ...
          sth_deg(fidx), meanErr),'FontSize',12);

    sgtitle(fig, 'Polar \rightarrow Cartesian:  how the first-order linearization breaks down', ...
            'FontSize',14,'FontWeight','bold');

    drawnow;
    frame = getframe(fig);
    [A,cm] = rgb2ind(frame2im(frame),256);
    if fidx==1
        imwrite(A,cm,gifOut,'gif','LoopCount',Inf,'DelayTime',0.12);
    else
        imwrite(A,cm,gifOut,'gif','WriteMode','append','DelayTime',0.12);
    end
end
try, exportgraphics(fig,pngOut,'Resolution',130); catch, saveas(fig,pngOut); end
fprintf('sigma_theta = %.0f deg (final)\n', sth_deg(end));
fprintf('MC   mean = [%.3f %.3f]\n', muMC);
fprintf('LIN  mean = [%.3f %.3f]   (g(mu))\n', muL);
fprintf('mean error = %.3f m,   MC radius bias = %.3f m\n', norm(muMC-muL), r0-norm(muMC));
fprintf('Saved GIF -> %s\n', gifOut);
fprintf('Saved PNG -> %s\n', pngOut);

%% ---- ellipse helper ----
function h = draw_ellipse(ax, mu, C, ns, col)
    C=(C+C')/2; [V,D]=eig(C); D=max(D,0);
    t=linspace(0,2*pi,90);
    e=V*sqrt(D)*[cos(t);sin(t)]*ns;
    h=plot(ax, mu(1)+e(1,:), mu(2)+e(2,:), '-','Color',col,'LineWidth',2.2);
end
