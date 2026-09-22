%% Symmetric set  ->  sigma points on the covariance ellipse (2D)
%  SEL5917 — Unscented Transformation (slide 51)
%  Symmetric set generator in R^n:
%     S = { 0, +/-e_1, +/-e_2, ..., +/-e_n },   #S = 2n+1
%  Sigma points:   chi^i = m + c * Sigma^(1/2) * s^i,   s^i in S,   c = sqrt(n+kappa)
%  In 2D (n=2) there are 2n+1 = 5 points: the centre plus a +/- pair along each
%  principal axis of the covariance ellipse.
%
%  Output: ../images/sigma_symmetric_set.png
%  Author: generated for Prof. Marcos R. Fernandes.

clear; clc; close all; rng(1);

n = 2;  kappa = 1;  c = sqrt(n+kappa);      % c = sqrt(n+kappa)
% symmetric set S (columns): centre, then +/- along each axis
S = [ 0  1 -1  0  0 ;
      0  0  0  1 -1 ];
lab = {'\chi^0','\chi^1','\chi^2','\chi^3','\chi^4'};

% a Gaussian with correlation (tilted ellipse)
m     = [3; 2];
Sigma = [3.0 1.4; 1.4 1.2];
[V,D] = eig(Sigma);   L = V*sqrt(D);        % valid Sigma^(1/2):  L*L' = Sigma

co = get(groot,'defaultAxesColorOrder');    % MATLAB default colors
cE = co(1,:);   % blue  - sigma-point ellipse
cP = co(2,:);   % orange- sigma points
cA = [0.5 0.5 0.5];

fig = figure('Color','w','Position',[80 80 1240 560]);

%% ---------- left: symmetric set in standard coordinates ----------
ax1 = subplot(1,2,1); hold(ax1,'on'); axis(ax1,'equal'); box(ax1,'on'); grid(ax1,'on');
draw_ellipse(ax1,[0;0],eye(2),1,cA,'--');        % unit circle (1-sigma of N(0,I))
draw_ellipse(ax1,[0;0],eye(2),c,cE,'-');         % c-circle where the set sits
plot(ax1,[-c c NaN 0 0],[0 0 NaN -c c],':','Color',cA);   % axes
Sp = c*S;
plot(ax1,Sp(1,2:end),Sp(2,2:end),'o','MarkerFaceColor',cP,'MarkerEdgeColor',cP,'MarkerSize',11);
plot(ax1,0,0,'s','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',11);
for i=1:5, text(ax1,Sp(1,i)+0.12,Sp(2,i)+0.12,['s^' num2str(i-1)],'FontSize',12); end
title(ax1,'Symmetric set  \it S \rm (standard coords)','FontSize',13);
xlabel(ax1,'\xi_1'); ylabel(ax1,'\xi_2');
axis(ax1,[-1 1 -1 1]*(c+0.8));
text(ax1,0,-(c+0.55),'#\it S \rm = 2n+1 = 5','HorizontalAlignment','center','FontSize',12);

%% ---------- right: sigma points on the covariance ellipse ----------
ax2 = subplot(1,2,2); hold(ax2,'on'); axis(ax2,'equal'); box(ax2,'on'); grid(ax2,'on');
% context samples
Xs = m + L*randn(2,600);
scatter(ax2,Xs(1,:),Xs(2,:),6,[0.7 0.7 0.72],'filled','MarkerFaceAlpha',0.35);
% covariance ellipses
draw_ellipse(ax2,m,Sigma,1,cA,'--');             % 1-sigma
draw_ellipse(ax2,m,Sigma,2,[0.8 0.8 0.82],'-');  % 2-sigma
draw_ellipse(ax2,m,Sigma,c,cE,'-');              % sigma-point ellipse (radius c)
% principal axes
for j=1:2
    d = c*sqrt(D(j,j))*V(:,j);
    plot(ax2,[m(1)-d(1) m(1)+d(1)],[m(2)-d(2) m(2)+d(2)],':','Color',cA);
end
% sigma points  chi = m + c*L*s
Chi = m + c*L*S;
plot(ax2,Chi(1,2:end),Chi(2,2:end),'o','MarkerFaceColor',cP,'MarkerEdgeColor',cP,'MarkerSize',12);
plot(ax2,m(1),m(2),'s','MarkerFaceColor','k','MarkerEdgeColor','k','MarkerSize',12);
for i=1:5, text(ax2,Chi(1,i)+0.15,Chi(2,i)+0.18,lab{i},'FontSize',13); end
title(ax2,'Sigma points on the covariance ellipse','FontSize',13);
xlabel(ax2,'x_1'); ylabel(ax2,'x_2');
axis(ax2,[m(1)-5.5 m(1)+5.5 m(2)-4.5 m(2)+4.5]);
legend(ax2, findobj(ax2,'Type','line','Marker','o'), ...
       {'sigma points  \chi^i'} ,'Location','southoutside','Box','off','FontSize',11);

sgtitle(fig, sprintf(['Unscented Transformation — symmetric sigma points   ' ...
        '(\\chi^i = m + \\surd(n+\\kappa)\\,\\Sigma^{1/2} s^i,  \\kappa=%g)'],kappa), ...
        'FontSize',14,'FontWeight','bold');

pngOut = fullfile(fileparts(mfilename('fullpath')),'..','images','sigma_symmetric_set.png');
try, exportgraphics(fig,pngOut,'Resolution',140); catch, saveas(fig,pngOut); end
fprintf('c = sqrt(n+kappa) = %.4f\n', c);
fprintf('Saved PNG -> %s\n', pngOut);

%% ---- ellipse helper (ns-sigma) ----
function draw_ellipse(ax, mu, C, ns, col, ls)
    C=(C+C')/2; [V,D]=eig(C); D=max(D,0);
    t=linspace(0,2*pi,120);
    e=V*sqrt(D)*[cos(t);sin(t)]*ns;
    plot(ax, mu(1)+e(1,:), mu(2)+e(2,:), ls,'Color',col,'LineWidth',2.0);
end
