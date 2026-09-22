%% BEARING-ONLY Target Tracking — EKF vs UKF  (the UKF clearly wins)
%  SEL5917 — Nonlinear Filtering
%  Same CT dynamics as slides 32-33, but the radar reports ONLY the bearing:
%      y_k = atan2(py_k, px_k) + eps_k        (range is NOT measured)
%  Range is weakly observable, the geometry is strongly nonlinear, and the EKF
%  linearization typically DIVERGES while the UKF stays consistent.
%  Output: ../images/ekf_ukf_bearing.gif , .png   (+ RMSE printout)
%  Author: generated for Prof. Marcos R. Fernandes.

clear; clc; close all; rng(7);
T=1.0; N=60; sig_th=deg2rad(1.5); R=sig_th^2;
q_v=0.25^2; q_Om=deg2rad(1.2)^2; Q=diag([1e-4,1e-4,q_v,1e-6,q_Om]);

%% truth
x=zeros(5,N); x(:,1)=[95;60;6.0;deg2rad(120);deg2rad(3.8)];
for k=1:N-1
    if k==16, x(5,k)=deg2rad(-4.2); elseif k==34, x(5,k)=deg2rad(3.6); elseif k==50, x(5,k)=deg2rad(-2.4); end
    x(:,k+1)=f_ct(x(:,k),T)+[0;0;sqrt(q_v)*randn;0;sqrt(q_Om)*randn];
end
%% bearing-only measurements
y=zeros(1,N); for k=1:N, y(k)=atan2(x(2,k),x(1,k))+sig_th*randn; end

%% shared init
x0=x(:,1)+[15;-15;1.5;deg2rad(15);deg2rad(2)];
P0=diag([25^2,25^2,3^2,deg2rad(28)^2,deg2rad(5)^2]);

%% EKF
xe=zeros(5,N); xe(:,1)=x0; P=P0; I5=eye(5);
for k=1:N-1
    A=jac_f(xe(:,k),T); xp=f_ct(xe(:,k),T); Pp=A*P*A'+Q;
    px=xp(1);py=xp(2); r2=px^2+py^2; H=[-py/r2 px/r2 0 0 0];
    inn=wrapToPi(y(k+1)-atan2(py,px)); S=H*Pp*H'+R; Kk=Pp*H'/S;
    xe(:,k+1)=xp+Kk*inn; xe(4,k+1)=wrapToPi(xe(4,k+1)); P=(I5-Kk*H)*Pp;
end
%% UKF
n=5; kappa=1; c=sqrt(n+kappa); wm=[kappa/(n+kappa) repmat(1/(2*(n+kappa)),1,2*n)]; wc=wm;
xu=zeros(5,N); xu(:,1)=x0; P=P0;
for k=1:N-1
    Ls=chol(P,'lower'); X=[xu(:,k) xu(:,k)+c*Ls xu(:,k)-c*Ls];
    Xp=zeros(5,2*n+1); for i=1:2*n+1, Xp(:,i)=f_ct(X(:,i),T); end
    mp=Xp*wm'; mp(4)=atan2(sum(wm.*sin(Xp(4,:))),sum(wm.*cos(Xp(4,:))));
    dX=Xp-mp; dX(4,:)=wrapToPi(dX(4,:)); Pp=(wc.*dX)*dX'+Q;
    Yp=atan2(Xp(2,:),Xp(1,:)); yh=atan2(sum(wm.*sin(Yp)),sum(wm.*cos(Yp)));
    dY=wrapToPi(Yp-yh); Sy=(wc.*dY)*dY'+R; Pxy=(wc.*dX)*dY';
    Kk=Pxy/Sy; xu(:,k+1)=mp+Kk*wrapToPi(y(k+1)-yh); xu(4,k+1)=wrapToPi(xu(4,k+1));
    P=Pp-Kk*Sy*Kk'; P=(P+P')/2;
end
rmse=@(a,b) sqrt(mean(sum((a-b).^2,1)));
posE=rmse(xe(1:2,:),x(1:2,:)); posU=rmse(xu(1:2,:),x(1:2,:));
fprintf('BEARING-ONLY position RMSE:  EKF = %.1f m | UKF = %.1f m\n',posE,posU);

%% animation
here=fileparts(mfilename('fullpath'));
gifOut=fullfile(here,'..','images','ekf_ukf_bearing.gif');
pngOut=fullfile(here,'..','images','ekf_ukf_bearing.png');
co=get(groot,'defaultAxesColorOrder');
cTrue=[0.12 0.12 0.14]; cEKF=co(2,:); cUKF=co(4,:); cK=[0.16 0.17 0.20]; cRot=[0.20 0.50 0.90]; cRay=[0.55 0.78 0.55];
allx=[x(1,:) xu(1,:) 0]; ally=[x(2,:) xu(2,:) 0];
cxc=(min(allx)+max(allx))/2; cyc=(min(ally)+max(ally))/2;
half=max(range(allx),range(ally))/2+30; LIM=[cxc-half cxc+half cyc-half cyc+half]; hs=half*0.05; Rray=3*half;

fig=figure('Color','w','Position',[100 100 830 780]); first=true;
for k=1:N
    clf; ax=axes(fig); hold(ax,'on'); axis(ax,'equal'); axis(ax,LIM); grid(ax,'on'); box(ax,'on');
    for rr=40:40:3*half, th=linspace(0,2*pi,90); plot(ax,rr*cos(th),rr*sin(th),'-','Color',[0.93 0.93 0.95]); end
    % bearing measurement rays so far (faint) + current (bold)
    for j=max(1,k-6):k
        al=0.12+0.5*(j-(k-6))/6; if j==k, al=0.9; end
        plot(ax,[0 Rray*cos(y(j))],[0 Rray*sin(y(j))],'-','Color',[cRay al],'LineWidth',1.0+(j==k));
    end
    draw_radar(ax,cK,hs);
    plot(ax,x(1,1:k),x(2,1:k),'-','Color',cTrue,'LineWidth',2.2);
    plot(ax,xe(1,1:k),xe(2,1:k),'--','Color',cEKF,'LineWidth',2.0);
    plot(ax,xu(1,1:k),xu(2,1:k),'-','Color',cUKF,'LineWidth',2.2);
    plot(ax,xe(1,k),xe(2,k),'s','Color',cEKF,'MarkerFaceColor',cEKF,'MarkerSize',7);
    plot(ax,xu(1,k),xu(2,k),'o','Color',cUKF,'MarkerFaceColor',cUKF,'MarkerSize',7);
    draw_hexa(ax,x(1:2,k),x(4,k),hs,cK,cRot);
    title(ax,sprintf('Bearing-only tracking   |   t = %2.0f s   |   pos-RMSE:  EKF %.0f m,  UKF %.0f m',(k-1)*T,posE,posU),'FontSize',12);
    xlabel(ax,'p_x  [m]'); ylabel(ax,'p_y  [m]');
    drawnow; fr=getframe(fig); [A,cm]=rgb2ind(frame2im(fr),256);
    if first, imwrite(A,cm,gifOut,'gif','LoopCount',Inf,'DelayTime',0.09); first=false;
    else,     imwrite(A,cm,gifOut,'gif','WriteMode','append','DelayTime',0.09); end
end
try, exportgraphics(fig,pngOut,'Resolution',130); catch, saveas(fig,pngOut); end
fprintf('Saved GIF -> %s\nSaved PNG -> %s\n',gifOut,pngOut);

%% locals
function xn=f_ct(x,T)
    px=x(1);py=x(2);v=x(3);ph=x(4);Om=x(5);
    if abs(Om)<1e-6, px2=px+v*T*cos(ph); py2=py+v*T*sin(ph);
    else, px2=px+(v/Om)*(sin(ph+Om*T)-sin(ph)); py2=py+(v/Om)*(-cos(ph+Om*T)+cos(ph)); end
    xn=[px2;py2;v;ph+Om*T;Om];
end
function A=jac_f(x,T), m=numel(x);A=zeros(m);h=1e-6; for j=1:m, d=zeros(m,1);d(j)=h; A(:,j)=(f_ct(x+d,T)-f_ct(x-d,T))/(2*h); end, end
function a=wrapToPi(a), a=mod(a+pi,2*pi)-pi; end
function draw_radar(ax,c,s)
    plot(ax,0,0,'^','MarkerSize',12,'MarkerFaceColor',c,'MarkerEdgeColor',c);
    plot(ax,[0 0],[0 s],'-','Color',c,'LineWidth',1.5);
    th=linspace(-pi/3,pi/3,20); plot(ax,0.85*s*sin(th),s+0.42*s*(cos(th)-1),'-','Color',c,'LineWidth',1.5);
    text(ax,0.6*s,-s,'RADAR','Color',c,'FontSize',9,'FontWeight','bold');
end
function draw_hexa(ax,c,hd,s,cArm,cRot)
    c=c(:);
    for a=0:60:300
        ang=deg2rad(a)+hd; d=c+s*[cos(ang);sin(ang)];
        plot(ax,[c(1) d(1)],[c(2) d(2)],'-','Color',cArm,'LineWidth',2);
        th=linspace(0,2*pi,18); patch(ax,d(1)+0.42*s*cos(th),d(2)+0.42*s*sin(th),cRot,'FaceAlpha',0.35,'EdgeColor',cRot,'LineWidth',1);
    end
    th=linspace(0,2*pi,18); patch(ax,c(1)+0.34*s*cos(th),c(2)+0.34*s*sin(th),cArm,'EdgeColor',cArm);
end
