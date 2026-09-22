%% Polar -> Cartesian : Monte-Carlo vs Taylor (EKF) vs Unscented Transform (UT)
%  SEL5917 — Nonlinear Filtering
%  Gaussian in polar coords (r,theta) pushed through g(r,theta)=[r cos, r sin].
%  Three approximations of the transformed mean/covariance:
%    (MC)     Monte-Carlo  (ground truth)                  -- blue
%    (TAYLOR) 1st-order Taylor / EKF:  mu=g(mu), C=G*Sig*G' -- orange
%    (UT)     Unscented:  sigma pts chi^i = mu + c*Sig^{1/2}*s^i, weighted moments -- purple
%
%  Output: ../images/polar2cart_ut_comparison.gif  and  .png
%  Author: generated for Prof. Marcos R. Fernandes.

clear; clc; close all; rng(3);

r0=10; th0=deg2rad(70); sr=0.35; Nmc=4000;
z1=randn(Nmc,1); z2=randn(Nmc,1);              % fixed draws -> smooth morph
sth_deg=[linspace(3,34,34), 34*ones(1,8)];

% ----- UT weights / sigma set (n=2) -----
n=2; kappa=1; c=sqrt(n+kappa);
S=[0 1 -1 0 0; 0 0 0 1 -1];                    % symmetric set (2n+1 cols)
w=[kappa/(n+kappa), repmat(1/(2*(n+kappa)),1,2*n)];   % weights w0,w,...

co=get(groot,'defaultAxesColorOrder');
cMC=co(1,:); cTAY=co(2,:); cUT=co(4,:);        % blue / orange / purple

here=fileparts(mfilename('fullpath'));
gifOut=fullfile(here,'..','images','polar2cart_ut_comparison.gif');
pngOut=fullfile(here,'..','images','polar2cart_ut_comparison.png');

g=@(r,th)[r.*cos(th); r.*sin(th)];

fig=figure('Color','w','Position',[60 60 1320 600]);

for fidx=1:numel(sth_deg)
    sth=deg2rad(sth_deg(fidx));
    mu=[r0;th0]; Sig=diag([sr^2, sth^2]); L=sqrt(Sig);   % diagonal -> elementwise sqrt

    % --- Monte-Carlo ---
    r=r0+sr*z1; th=th0+sth*z2;
    P=[r.*cos(th), r.*sin(th)];
    muMC=mean(P,1)'; covMC=cov(P);

    % --- Taylor / EKF ---
    muT=g(r0,th0);
    G=[cos(th0), -r0*sin(th0); sin(th0), r0*cos(th0)];
    covT=G*Sig*G';

    % --- Unscented Transform ---
    Chi = mu + c*L*S;                          % sigma points in polar space (2 x 5)
    Y   = [Chi(1,:).*cos(Chi(2,:)); Chi(1,:).*sin(Chi(2,:))];  % transformed (2 x 5)
    muU = Y*w';
    dY  = Y - muU;
    covU= (w.*dY)*dY';

    errT=norm(muMC-muT); errU=norm(muMC-muU);

    clf(fig);
    % ---------- left: input Gaussian in polar space ----------
    ax1=subplot(1,3,1); hold(ax1,'on'); box(ax1,'on'); grid(ax1,'on');
    scatter(ax1, rad2deg(th), r, 6,[0.6 0.6 0.62],'filled','MarkerFaceAlpha',0.22);
    draw_ellipse(ax1,[rad2deg(th0);r0],diag([rad2deg(sth)^2, sr^2]),2,cMC,'-');
    % sigma points in polar space
    plot(ax1, rad2deg(Chi(2,:)), Chi(1,:), 'd','MarkerFaceColor',cUT,'MarkerEdgeColor','k','MarkerSize',9);
    xlabel(ax1,'bearing  \theta  [deg]'); ylabel(ax1,'range  r  [m]');
    title(ax1,'Input:  Gaussian + sigma points','FontSize',12);
    xlim(ax1,[th0*180/pi-105, th0*180/pi+105]); ylim(ax1,[r0-4*sr, r0+4*sr]);

    % ---------- right: Cartesian comparison ----------
    ax2=subplot(1,3,[2 3]); hold(ax2,'on'); box(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal');
    scatter(ax2, P(:,1),P(:,2), 6,[0.62 0.62 0.66],'filled','MarkerFaceAlpha',0.18);
    hMC =draw_ellipse(ax2,muMC,covMC,2,cMC ,'-');
    hTAY=draw_ellipse(ax2,muT ,covT ,2,cTAY,'-');
    hUT =draw_ellipse(ax2,muU ,covU ,2,cUT ,'-');
    % transformed sigma points
    plot(ax2, Y(1,:),Y(2,:),'d','MarkerFaceColor',cUT,'MarkerEdgeColor','k','MarkerSize',9);
    % means
    plot(ax2,muMC(1),muMC(2),'o','MarkerFaceColor',cMC ,'MarkerEdgeColor',cMC ,'MarkerSize',7);
    plot(ax2,muT(1) ,muT(2) ,'s','MarkerFaceColor',cTAY,'MarkerEdgeColor',cTAY,'MarkerSize',8);
    xlabel(ax2,'p_x  [m]'); ylabel(ax2,'p_y  [m]');
    xlim(ax2,[-7 10]); ylim(ax2,[1 12]);
    legend(ax2,[hMC hTAY hUT], ...
        {'Monte-Carlo (true)','1^{st}-order Taylor / EKF','Unscented (UT)'}, ...
        'Location','southwest','FontSize',10,'Box','off');
    title(ax2, sprintf('Cartesian:  \\sigma_\\theta = %2.0f^\\circ    |    mean error:  Taylor = %.2f m,  UT = %.2f m', ...
          sth_deg(fidx),errT,errU),'FontSize',12);

    sgtitle(fig,'Polar \rightarrow Cartesian:  Monte-Carlo  vs  Taylor (EKF)  vs  Unscented (UT)', ...
            'FontSize',14,'FontWeight','bold');

    drawnow;
    frame=getframe(fig); [A,cm]=rgb2ind(frame2im(frame),256);
    if fidx==1, imwrite(A,cm,gifOut,'gif','LoopCount',Inf,'DelayTime',0.12);
    else,       imwrite(A,cm,gifOut,'gif','WriteMode','append','DelayTime',0.12); end
end
try, exportgraphics(fig,pngOut,'Resolution',130); catch, saveas(fig,pngOut); end
fprintf('final: Taylor mean err = %.3f m,  UT mean err = %.3f m\n', errT, errU);
fprintf('Saved GIF -> %s\nSaved PNG -> %s\n', gifOut, pngOut);

%% ---- ellipse helper ----
function h=draw_ellipse(ax,mu,C,ns,col,ls)
    C=(C+C')/2; [V,D]=eig(C); D=max(D,0);
    t=linspace(0,2*pi,100); e=V*sqrt(D)*[cos(t);sin(t)]*ns;
    h=plot(ax, mu(1)+e(1,:), mu(2)+e(2,:), ls,'Color',col,'LineWidth',2.4);
end
