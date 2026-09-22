%% Filter tuning (Q,R) and consistency diagnostics — scalar KF
%  SEL5917 — Nonlinear Filtering / Estimation
%  A tiny scalar local-level (random-walk) model, filtered TWICE:
%     * GOOD tuning : Q,R matched to the truth  -> innovations are white,
%                     normalized innovations ~ N(0,1) inside the +/-3 band.
%     * BAD  tuning : Q,R far too small (over-confident filter) -> the filter
%                     lags, innovations become AUTOCORRELATED and the
%                     normalized innovations blow far past +/-3.
%  Produces THREE figures (one per slide):
%     ../images/tuning_residuals.png  — innovations e_k vs time
%     ../images/tuning_autocorr.png   — autocorrelation of e_k (whiteness)
%     ../images/tuning_nis.png        — normalized innovations e_k/sqrt(S_k), +/-3 sigma
%  Author: generated for Prof. Marcos R. Fernandes.

clear; clc; close all; rng(11);

%% ---- scalar model:  x_k = a x_{k-1} + w,   y_k = x_k + v ----
N   = 220;  a = 0.90;      % stable AR(1) -> stationary truth (bounded)
Qt  = 1.0;                 % true process-noise variance
Rt  = 4.0;                 % true measurement-noise variance
xt  = zeros(1,N); xt(1)=0;
for k=2:N, xt(k)=a*xt(k-1)+sqrt(Qt)*randn; end
y   = xt + sqrt(Rt)*randn(1,N);

%% ---- two filters: GOOD and BAD tuning ----
good = struct('Q',Qt,   'R',Rt);      % matched
bad  = struct('Q',0.02, 'R',Rt);      % Q ~50x too small -> filter cannot keep up

[eG,SG] = run_kf(y,a,good.Q,good.R);
[eB,SB] = run_kf(y,a,bad.Q ,bad.R );

% normalized innovations (should be ~ N(0,1))
zG = eG./sqrt(SG);   zB = eB./sqrt(SB);

% autocorrelation of the innovations
M = 30;
[acG,lags] = acf(eG,M);
[acB,~   ] = acf(eB,M);
cb = 1.96/sqrt(numel(eG));            % 95% white-noise band

%% ---- console consistency summary ----
fprintf('==== innovation consistency ====\n');
fprintf('GOOD:  mean(z)=%+.2f  var(z)=%.2f  %%|z|<3 = %.1f%%\n', mean(zG),var(zG),100*mean(abs(zG)<3));
fprintf('BAD :  mean(z)=%+.2f  var(z)=%.2f  %%|z|<3 = %.1f%%\n', mean(zB),var(zB),100*mean(abs(zB)<3));

here=fileparts(mfilename('fullpath')); imdir=fullfile(here,'..','images');
co=get(groot,'defaultAxesColorOrder');
cG=co(1,:); cB=co(2,:); c3=[0.85 0.10 0.10]; gr=[0.55 0.55 0.60];
tt=1:numel(eG);

%% ================= FIGURE 1 — innovations (residuals) =================
f1=figure('Color','w','Position',[100 100 1180 620]);
ax=subplot(2,1,1); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
plot(ax,tt,eG,'-','Color',cG,'LineWidth',1.6);
yline(ax,0,'-','Color',gr);
title(ax,sprintf('GOOD tuning:  Q = %.2f,  R = %.2f   (matched)',good.Q,good.R),'FontSize',13);
ylabel(ax,'innovation  e_k'); legend(ax,{'e_k'},'Location','northeast','Box','off');
axG=subplot(2,1,2); hold(axG,'on'); grid(axG,'on'); box(axG,'on');
plot(axG,tt,eB,'-','Color',cB,'LineWidth',1.6);
yline(axG,0,'-','Color',gr);
title(axG,sprintf('BAD tuning:  Q = %.2f,  R = %.2f   (Q too small \\rightarrow filter lags the state)',bad.Q,bad.R),'FontSize',13);
xlabel(axG,'time step  k'); ylabel(axG,'innovation  e_k');
legend(axG,{'e_k'},'Location','northeast','Box','off');
exportgraphics(f1,fullfile(imdir,'tuning_residuals.png'),'Resolution',140);

%% ================= FIGURE 2 — autocorrelation (whiteness) =================
f2=figure('Color','w','Position',[100 100 1180 620]);
ax=subplot(2,1,1); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
stem(ax,lags,acG,'filled','Color',cG,'MarkerSize',4,'LineWidth',1.2);
plot(ax,lags,cb+0*lags,'--','Color',c3,'LineWidth',1.2); plot(ax,lags,-cb+0*lags,'--','Color',c3,'LineWidth',1.2);
title(ax,'GOOD tuning: autocorrelation of e_k  \rightarrow  white (inside the 95% band)','FontSize',13);
ylabel(ax,'\rho(\tau)'); ylim(ax,[-0.5 1.05]); xlim(ax,[0 M]);
legend(ax,{'\rho(\tau)','95% band'},'Location','northeast','Box','off');
axB=subplot(2,1,2); hold(axB,'on'); grid(axB,'on'); box(axB,'on');
stem(axB,lags,acB,'filled','Color',cB,'MarkerSize',4,'LineWidth',1.2);
plot(axB,lags,cb+0*lags,'--','Color',c3,'LineWidth',1.2); plot(axB,lags,-cb+0*lags,'--','Color',c3,'LineWidth',1.2);
title(axB,'BAD tuning: strong positive autocorrelation  \rightarrow  residuals NOT white','FontSize',13);
xlabel(axB,'lag  \tau'); ylabel(axB,'\rho(\tau)'); ylim(axB,[-0.5 1.05]); xlim(axB,[0 M]);
legend(axB,{'\rho(\tau)','95% band'},'Location','northeast','Box','off');
exportgraphics(f2,fullfile(imdir,'tuning_autocorr.png'),'Resolution',140);

%% ================= FIGURE 3 — normalized innovations, +/-3 sigma =================
f3=figure('Color','w','Position',[100 100 1180 620]);
ax=subplot(2,1,1); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
plot(ax,tt,zG,'-','Color',cG,'LineWidth',1.4);
plot(ax,tt, 3+0*tt,'--','Color',c3,'LineWidth',1.3); plot(ax,tt,-3+0*tt,'--','Color',c3,'LineWidth',1.3);
title(ax,sprintf('GOOD tuning: normalized innovation z_k=e_k/\\surd{S_k}   (%.0f%% inside \\pm3\\sigma)',100*mean(abs(zG)<3)),'FontSize',13);
ylabel(ax,'z_k'); ylim(ax,[-6 6]);
legend(ax,{'z_k','\pm3\sigma'},'Location','northeast','Box','off');
axB=subplot(2,1,2); hold(axB,'on'); grid(axB,'on'); box(axB,'on');
plot(axB,tt,zB,'-','Color',cB,'LineWidth',1.4);
plot(axB,tt, 3+0*tt,'--','Color',c3,'LineWidth',1.3); plot(axB,tt,-3+0*tt,'--','Color',c3,'LineWidth',1.3);
title(axB,sprintf('BAD tuning: filter is over-confident  \\rightarrow  z_k escapes the band  (only %.0f%% inside \\pm3\\sigma)',100*mean(abs(zB)<3)),'FontSize',13);
xlabel(axB,'time step  k'); ylabel(axB,'z_k');
legend(axB,{'z_k','\pm3\sigma'},'Location','northeast','Box','off');
exportgraphics(f3,fullfile(imdir,'tuning_nis.png'),'Resolution',140);

fprintf('Saved 3 figures -> %s\n',imdir);

%% ===================== local functions =====================
function [e,S] = run_kf(y,a,Q,R)
    N=numel(y); e=zeros(1,N-1); S=zeros(1,N-1);
    xh=y(1); P=R;                     % reasonable start
    for k=2:N
        xp=a*xh;  Pp=a^2*P+Q;         % predict
        S(k-1)=Pp+R;                  % innovation covariance (H=1)
        e(k-1)=y(k)-xp;               % innovation / residual
        K=Pp/S(k-1);                  % gain
        xh=xp+K*e(k-1);  P=(1-K)*Pp;  % update
    end
end
function [ac,lags]=acf(x,M)
    x=x-mean(x); c0=sum(x.^2); ac=zeros(1,M+1);
    for l=0:M, ac(l+1)=sum(x(1:end-l).*x(1+l:end))/c0; end
    lags=0:M;
end
