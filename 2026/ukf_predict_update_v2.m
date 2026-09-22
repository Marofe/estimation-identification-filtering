%% UKF — didactic 2D animation of the Prediction and Update steps
%  SEL5917 — The Unscented Kalman Filter
%  The state x=[x1;x2] is the position of a hexacopter drone.
%  Only the 2n+1 = 5 sigma points are propagated (no Monte-Carlo cloud).
%  Story:  prior -> sigma points -> propagate through f (trails; drone flies its
%          true path) -> predicted mean/cov (+Q) -> regenerate update sigma points
%          -> measurement -> posterior (shown at once).
%  Output: ../images/ukf_predict_update.gif
%  Author: generated for Prof. Marcos R. Fernandes.

clear; clc; close all; rng(7);

%% ---- UKF weights & symmetric set (n=2) ----
n=2; kappa=1; c=sqrt(n+kappa);
S=[0 1 -1 0 0; 0 0 0 1 -1];
wm=[kappa/(n+kappa), repmat(1/(2*(n+kappa)),1,2*n)];  wc=wm;
lab={'\chi^0','\chi^1','\chi^2','\chi^3','\chi^4'};

%% ---- prior, noises ----
m0=[-1.6; 0.2];  P0=[0.55 0.18; 0.18 0.40];
Q =0.02*eye(2);  R=[0.35 0.05; 0.05 0.22];
L0=chol(P0,'lower');
X0 = m0 + c*L0*S;                     % prior sigma points (2 x 5)

%% ---- nonlinear dynamics: swirl about a pivot (radius-dependent rotation) ----
pv=[1.6;0]; thTot=1.15; beta=0.55; rref=norm(m0-pv);
rot=@(X,frac) pv + rotmap(X-pv, frac*thTot, beta, rref);

%% ---- true state & its path (drone) ----
true0 = m0 + [0.55; -0.35];           % true state (estimate has some error)
Ntraj=48; T3=zeros(2,5,Ntraj); Ttrue=zeros(2,Ntraj);
for t=1:Ntraj, T3(:,:,t)=rot(X0,t/Ntraj); Ttrue(:,t)=rot(true0,t/Ntraj); end
true1=Ttrue(:,end);

%% ---- predicted moments ----
Xp = rot(X0,1);  mp = Xp*wm';  dP=Xp-mp;  Pp=(wc.*dP)*dP' + Q;

%% ---- regenerate sigma points for the update, from N(mp,Pp) ----
Lp=chol(Pp,'lower');  Xu = mp + c*Lp*S;

%% ---- measurement (of true position) + update (H=I) ----
y  = true1 + chol(R,'lower')*randn(2,1);
K  = Pp/(Pp+R);  mu = mp + K*(y-mp);  Pu=(eye(2)-K)*Pp;

%% ---- limits & colors ----
allP=[X0, Xp, Xu, y, m0, mp, mu, true0, reshape(T3,2,[]), Ttrue];
cx=mean([min(allP(1,:)) max(allP(1,:))]); cy=mean([min(allP(2,:)) max(allP(2,:))]);
half=max(range(allP(1,:)),range(allP(2,:)))/2 + 1.1;
LIM=[cx-half cx+half cy-half cy+half];  hs=half*0.055;
co=get(groot,'defaultAxesColorOrder');
cPri=co(1,:); cSig=co(2,:); cMeas=co(5,:); cPost=co(4,:); edge=[0.4 0.4 0.4]; trailC=[0.96 0.78 0.55];
cArm=[0.15 0.15 0.18]; cRot=[0.20 0.50 0.90];

hd0 = atan2(Ttrue(2,1)-true0(2), Ttrue(1,1)-true0(1));
hdE = atan2(Ttrue(2,end)-Ttrue(2,end-1), Ttrue(1,end)-Ttrue(1,end-1));

fig=figure('Color','w','Position',[100 100 780 720]);
F={};

%% ===== Phase 1: prior + reveal sigma points; drone at true state =====
for k=1:10
    setupAx(LIM);
    draw_ellipse(m0,P0,2,cPri,'-');
    plot(m0(1),m0(2),'o','MarkerFaceColor',cPri,'MarkerEdgeColor','k','MarkerSize',9);
    nshow=min(5,max(0,k-3));
    for i=1:nshow, sigmaPt(X0(:,i),lab{i},cSig); end
    draw_hexa(true0,hd0,hs,cArm,cRot);
    text(true0(1)+1.2*hs,true0(2)-1.4*hs,'true state','FontSize',10,'Color',cArm);
    banner('Step 1  —  Prior', 'N(x_{k|k}, P_{k|k}):  build 2n+1 = 5 sigma points  \chi^i', cPri);
    F{end+1}=snap(fig);
end

%% ===== Phase 2: propagate ONLY the sigma points (trails) + drone flies path =====
for k=1:Ntraj
    setupAx(LIM);
    draw_ellipse(m0,P0,2,cPri,'--');
    for i=1:5, plot(squeeze(T3(1,i,1:k)),squeeze(T3(2,i,1:k)),'-','Color',trailC,'LineWidth',1.3); end
    plot(Ttrue(1,1:k),Ttrue(2,1:k),'-','Color',cArm,'LineWidth',1.6);      % true state path
    Xc=T3(:,:,k);
    for i=1:5, sigmaPt(Xc(:,i),lab{i},cSig); end
    hd = atan2(Ttrue(2,k)-Ttrue(2,max(1,k-1)), Ttrue(1,k)-Ttrue(1,max(1,k-1)));
    draw_hexa(Ttrue(:,k),hd,hs,cArm,cRot);
    banner('Step 2  —  Propagate', 'push every sigma point through f;  the drone flies its true path', cSig);
    F{end+1}=snap(fig);
end

%% ===== Phase 3: predicted mean & covariance =====
for k=1:10
    setupAx(LIM);
    for i=1:5, plot(squeeze(T3(1,i,:)),squeeze(T3(2,i,:)),'-','Color',trailC,'LineWidth',1.0); end
    plot(Ttrue(1,:),Ttrue(2,:),'-','Color',cArm,'LineWidth',1.6);
    for i=1:5, sigmaPt(Xp(:,i),lab{i},cSig); end
    draw_ellipse(mp,Pp,2,cPri,'-');
    plot(mp(1),mp(2),'o','MarkerFaceColor',cPri,'MarkerEdgeColor','k','MarkerSize',9);
    draw_hexa(true1,hdE,hs,cArm,cRot);
    banner('Step 3  —  Prediction', 'weighted mean & covariance (+Q):  N(x_{k+1|k}, P_{k+1|k})', cPri);
    F{end+1}=snap(fig);
end

%% ===== Phase 4: regenerate the sigma points for the UPDATE =====
for k=1:10
    setupAx(LIM);
    plot(Ttrue(1,:),Ttrue(2,:),'-','Color',cArm,'LineWidth',1.2);
    draw_ellipse(mp,Pp,2,cPri,'-'); plot(mp(1),mp(2),'o','MarkerFaceColor',cPri,'MarkerEdgeColor','k','MarkerSize',9);
    for i=1:5, sigmaPt(Xu(:,i),lab{i},cSig); end
    draw_hexa(true1,hdE,hs,cArm,cRot);
    banner('Step 4  —  Update sigma points', 'regenerate  \chi^i  from  N(x_{k+1|k}, P_{k+1|k})  and pass through h  (H=I)', cSig);
    F{end+1}=snap(fig);
end

%% ===== Phase 5: measurement arrives =====
for k=1:8
    setupAx(LIM);
    plot(Ttrue(1,:),Ttrue(2,:),'-','Color',cArm,'LineWidth',1.2);
    draw_ellipse(mp,Pp,2,cPri,'-'); plot(mp(1),mp(2),'o','MarkerFaceColor',cPri,'MarkerEdgeColor','k','MarkerSize',9);
    for i=1:5, sigmaPt(Xu(:,i),'',cSig); end
    draw_ellipse(y,R,2,cMeas,'--'); plot(y(1),y(2),'x','Color',cMeas,'MarkerSize',15,'LineWidth',3);
    draw_hexa(true1,hdE,hs,cArm,cRot);
    banner('Step 5  —  Measurement', 'a noisy measurement  y  of the drone position  (covariance R)', cMeas);
    F{end+1}=snap(fig);
end

%% ===== Phase 6: posterior appears instantly, then hold =====
for k=1:18
    setupAx(LIM);
    plot(Ttrue(1,:),Ttrue(2,:),'-','Color',cArm,'LineWidth',1.2);
    draw_ellipse(mp,Pp,2,cPri,'--');
    for i=1:5, sigmaPt(Xu(:,i),'',cSig); end
    draw_ellipse(y,R,2,cMeas,'--'); plot(y(1),y(2),'x','Color',cMeas,'MarkerSize',15,'LineWidth',3);
    plot([mp(1) mu(1)],[mp(2) mu(2)],'-','Color',edge);
    draw_ellipse(mu,Pu,2,cPost,'-');
    plot(mu(1),mu(2),'o','MarkerFaceColor',cPost,'MarkerEdgeColor','k','MarkerSize',10);
    draw_hexa(true1,hdE,hs,cArm,cRot);
    text(LIM(1)+0.15,LIM(4)-0.4, ...
       '\color[rgb]{0,0.447,0.741}prediction    \color[rgb]{0.466,0.674,0.188}measurement    \color[rgb]{0.494,0.184,0.556}posterior', ...
       'FontSize',11,'FontWeight','bold');
    banner('Step 6  —  Update / Posterior', 'K = P_{xy}S^{-1};   posterior  N(x_{k+1|k+1}, P_{k+1|k+1})', cPost);
    F{end+1}=snap(fig);
end

%% ---- write GIF ----
gifOut=fullfile(fileparts(mfilename('fullpath')),'..','images','ukf_predict_update.gif');
for i=1:numel(F)
    [A,cm]=rgb2ind(F{i},256);
    if i==1, imwrite(A,cm,gifOut,'gif','LoopCount',Inf,'DelayTime',0.08);
    else,    imwrite(A,cm,gifOut,'gif','WriteMode','append','DelayTime',0.08); end
end
fprintf('Wrote %d frames -> %s\n', numel(F), gifOut);

%% ================= local functions =================
function im = snap(fig), drawnow; im = frame2im(getframe(fig)); end
function setupAx(LIM)
    cla; hold on; box on; grid on; axis equal; axis(LIM);
    set(gca,'FontSize',11); xlabel('x_1'); ylabel('x_2');
end
function banner(step,txt,col)
    title({['\bf' step],['\rm\fontsize{12}' txt]},'Color',col,'FontSize',15);
end
function sigmaPt(p,txt,col)
    plot(p(1),p(2),'d','MarkerFaceColor',col,'MarkerEdgeColor','k','MarkerSize',11);
    if ~isempty(txt), text(p(1)+0.12,p(2)+0.14,txt,'FontSize',11,'Color',[0.15 0.15 0.15]); end
end
function draw_hexa(c,hd,s,cArm,cRot)
    c=c(:);
    for a=0:60:300
        ang=deg2rad(a)+hd; d=c+s*[cos(ang);sin(ang)];
        plot([c(1) d(1)],[c(2) d(2)],'-','Color',cArm,'LineWidth',2);
        th=linspace(0,2*pi,20);
        patch(d(1)+0.42*s*cos(th), d(2)+0.42*s*sin(th), cRot,'FaceAlpha',0.35,'EdgeColor',cRot,'LineWidth',1.1);
    end
    th=linspace(0,2*pi,20);
    patch(c(1)+0.34*s*cos(th), c(2)+0.34*s*sin(th), cArm,'EdgeColor',cArm);
end
function Y = rotmap(U, ang, beta, rref)
    r = sqrt(sum(U.^2,1));  a = ang.*(1 + beta*(r-rref));
    Y = [cos(a).*U(1,:) - sin(a).*U(2,:);  sin(a).*U(1,:) + cos(a).*U(2,:)];
end
function draw_ellipse(mu,C,ns,col,ls)
    C=(C+C')/2; [V,D]=eig(C); D=max(D,0);
    t=linspace(0,2*pi,100); e=V*sqrt(D)*[cos(t);sin(t)]*ns;
    plot(mu(1)+e(1,:), mu(2)+e(2,:), ls,'Color',col,'LineWidth',2.4);
end
