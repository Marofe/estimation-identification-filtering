%% EKF vs UKF — classic 1-D nonlinear benchmark (Kitagawa / Gordon-Salmond-Smith)
%  SEL5917 — Nonlinear Filtering
%    x_k = 0.5 x_{k-1} + 25 x_{k-1}/(1+x_{k-1}^2) + 8 cos(1.2 k) + w_k,  w~N(0,Q)
%    y_k = x_k^2 / 20 + v_k,                                            v~N(0,R)
%  Strongly nonlinear dynamics AND a sign-blind (x^2) measurement: the textbook
%  case where the EKF struggles/diverges and the UKF tracks much better.
%  Output: ../images/benchmark1d.png  and prints the RMSE comparison.
%  Author: generated for Prof. Marcos R. Fernandes.

clear; clc; close all; rng(5);

K=50; Q=10; R=1;
f=@(x,k) 0.5*x + 25*x./(1+x.^2) + 8*cos(1.2*k);
df=@(x)  0.5 + 25*(1-x.^2)./(1+x.^2).^2;      % df/dx
h=@(x)   x.^2/20;
dh=@(x)  x/10;                                 % dh/dx

%% ---- ground truth + measurements ----
xt=zeros(1,K); xt(1)=0.1;
for k=2:K, xt(k)=f(xt(k-1),k-1)+sqrt(Q)*randn; end
y=zeros(1,K); for k=1:K, y(k)=h(xt(k))+sqrt(R)*randn; end

%% ---- shared init ----
m0=0; P0=5;

%% ---- EKF ----
xe=zeros(1,K); xe(1)=m0; P=P0;
for k=2:K
    A=df(xe(k-1)); xp=f(xe(k-1),k-1); Pp=A*P*A+Q;
    H=dh(xp); S=H*Pp*H+R; Kk=Pp*H/S;
    xe(k)=xp+Kk*(y(k)-h(xp)); P=(1-Kk*H)*Pp;
end

%% ---- UKF (n=1) ----
n=1; kappa=2; c=sqrt(n+kappa);
wm=[kappa/(n+kappa), 1/(2*(n+kappa)), 1/(2*(n+kappa))]; wc=wm;
xu=zeros(1,K); xu(1)=m0; P=P0;
for k=2:K
    sp=sqrt(P); X=[xu(k-1), xu(k-1)+c*sp, xu(k-1)-c*sp];
    Xp=f(X,k-1);                       mp=Xp*wm';  dX=Xp-mp; Pp=(wc.*dX)*dX'+Q;
    Yp=h(Xp);                          yh=Yp*wm';  dY=Yp-yh; Sy=(wc.*dY)*dY'+R;
    Pxy=(wc.*dX)*dY'; Kk=Pxy/Sy;
    xu(k)=mp+Kk*(y(k)-yh); P=Pp-Kk*Sy*Kk;
end

%% ---- RMSE ----
rmseE=sqrt(mean((xe-xt).^2)); rmseU=sqrt(mean((xu-xt).^2));
fprintf('==== 1-D benchmark RMSE (K=%d) ====\n',K);
fprintf('EKF = %.2f   |   UKF = %.2f\n',rmseE,rmseU);

%% ---- figure ----
co=get(groot,'defaultAxesColorOrder');
cT=[0.12 0.12 0.14]; cE=co(2,:); cU=co(4,:);
fig=figure('Color','w','Position',[100 100 1180 640]);

ax1=subplot(2,1,1); hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
plot(ax1,1:K,xt,'-','Color',cT,'LineWidth',2.4);
plot(ax1,1:K,xe,'--','Color',cE,'LineWidth',2.0);
plot(ax1,1:K,xu,'-','Color',cU,'LineWidth',2.0);
ylabel(ax1,'state  x_k'); ylim(ax1,[-32 32]);
legend(ax1,{'true','EKF','UKF'},'Location','northoutside','Orientation','horizontal','FontSize',12,'Box','off');
title(ax1,sprintf('1-D nonlinear benchmark:  RMSE  EKF = %.1f   vs   UKF = %.1f',rmseE,rmseU),'FontSize',14,'FontWeight','bold');

ax2=subplot(2,1,2); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
plot(ax2,1:K,abs(xe-xt),'--','Color',cE,'LineWidth',2.0);
plot(ax2,1:K,abs(xu-xt),'-','Color',cU,'LineWidth',2.0);
xlabel(ax2,'time step  k'); ylabel(ax2,'|error|');
legend(ax2,{'EKF error','UKF error'},'Location','northeast','FontSize',11,'Box','off');

pngOut=fullfile(fileparts(mfilename('fullpath')),'..','images','benchmark1d.png');
try, exportgraphics(fig,pngOut,'Resolution',140); catch, saveas(fig,pngOut); end
fprintf('Saved PNG -> %s\n',pngOut);
