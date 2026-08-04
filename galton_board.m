function galton_board
% GALTON_BOARD  Realistic Galton board (bean machine / quincunx) simulation.
%
%   A ball is released above a triangular array of n peg rows. At each peg it
%   bounces RIGHT with probability p and LEFT with probability 1-p, so the bin
%   it finally lands in equals its number of right bounces ~ Binomial(n, p).
%   Balls physically PILE UP in the collecting bins right below the pegs, so
%   the heap of stacked balls IS the histogram. As n grows the profile tends
%   to the Normal curve N(np, np(1-p))  (De Moivre-Laplace / CLT).
%
%   USAGE
%       >> galton_board            % just run it; edit the PARAMETERS block
%
%   No toolboxes required (base MATLAB only).
%
%   SEL5917 - Estimation, Identification and Stochastic Filtering
% -------------------------------------------------------------------------

% ============================ PARAMETERS =================================
n           = 12;     % number of peg rows            (=> n+1 bins, k = 0..n)
N           = 500;    % number of balls to drop
p           = 0.5;    % probability of bouncing right at each peg (0<p<1)
ballsAcross = 3;      % how many balls sit side-by-side in a bin (pile width)
animate     = true;   % true: watch the balls fall & pile up; false: instant
launchRate  = 3;      % new balls released per frame (=> many falling at once)
frameDelay  = 0.0;    % extra pause per animation frame, seconds (0 = fastest)
showNormal  = true;   % overlay the theoretical Normal curve on the heap
seed        = 0;      % RNG seed for reproducibility ([] = random each run)
makeGif     = true;   % also save an animated GIF of the run
gifFile     = 'galton_board.gif';   % output GIF (written to the current folder)
gifEvery    = 6;      % capture one GIF frame every this many balls
gifDelay    = 0.06;   % seconds between GIF frames
% ========================================================================

if ~isempty(seed), rng(seed); end
if p <= 0 || p >= 1, error('galton_board:p','p must be strictly between 0 and 1.'); end

% ------------------------------ geometry --------------------------------
pegGap = 1;                         % horizontal spacing between pegs / bin width
ballD  = 0.98 * pegGap / ballsAcross;   % ball diameter
layerH = 0.87 * ballD;              % vertical spacing between stacked layers
floorY = 0;                         % bins rest on this floor

kk     = 0:n;                       % bin indices
% Binomial(n,p) pmf via gammaln (toolbox-free, overflow-safe) -> size the board
logpmf = gammaln(n+1) - gammaln(kk+1) - gammaln(n-kk+1) + kk.*log(p) + (n-kk).*log(1-p);
pmf    = exp(logpmf);
maxLayers = ceil(max(pmf)*N / ballsAcross);
binTopY   = floorY + max(maxLayers,1)*layerH + 3*ballD;   % top of the bin walls

pegRowH  = 1;                                  % vertical gap between peg rows
pegBaseY = binTopY + 1;                        % lowest (widest) peg row height
releaseY = pegBaseY + (n-1)*pegRowH + 1.5;     % ball release height
bx       = (kk - n/2) * pegGap;                % bin centre x for each k
xEdges   = ((-n/2):1:(n/2+1)) - 0.5;           % vertical bin-wall positions
xEdges   = [xEdges(1)-1, xEdges];              % pad so k=0..n all get walls
cnt      = zeros(1, n+1);                      % running ball count per bin

% ------------------------------ figure ----------------------------------
hFig = figure('Color','w','Name','Galton board','NumberTitle','off', ...
              'Position',[120 40 780 860]);
ax = axes('Position',[0.04 0.04 0.92 0.92]); hold(ax,'on'); axis(ax,'equal');
set(ax,'XTick',[],'YTick',[],'XColor','none','YColor','none');
xlim(ax, [-(n/2+2)*pegGap, (n/2+2)*pegGap]);
ylim(ax, [floorY-1, releaseY+1]);
title(ax, sprintf('Galton board   (N=%d, n=%d, p=%.2f)', N, n, p), 'FontWeight','bold');

drawBins(ax, xEdges, floorY, binTopY);
drawPegs(ax, n, pegGap, pegBaseY, pegRowH);

% single "active" ball (a real, data-scaled circle) reused for every drop
% ---- a reusable pool of in-flight balls (many can fall at the same time) ----
settleSteps = 4;                                   % frames to drop into the heap
poolSize    = launchRate*(n + settleSteps) + launchRate + 2;
movers      = gobjects(1, poolSize);
for i = 1:poolSize
    movers(i) = rectangle(ax,'Position',[-ballD/2, releaseY-ballD/2, ballD, ballD], ...
                'Curvature',[1 1],'FaceColor',[0.85 0.10 0.10], ...
                'EdgeColor','k','LineWidth',0.6,'Visible','off');
end
slotBusy = false(1, poolSize);      % is pool slot i carrying a ball?
slotS    = zeros(1, poolSize);      % step index of that ball (1..n+settleSteps)
slotPX   = zeros(poolSize, n);      % that ball's x after each bounce
slotXr   = zeros(1, poolSize);      % its target x in the heap
slotYr   = zeros(1, poolSize);      % its target y in the heap

if makeGif, drawnow; captureGif(hFig, gifFile, gifDelay, true); end   % empty board

if animate
    launched = 0; settled = 0; frame = 0;
    while settled < N
        frame = frame + 1;

        % ---- release up to launchRate new balls into free slots ----
        for L = 1:launchRate
            if launched >= N, break; end
            slot = find(~slotBusy, 1);
            if isempty(slot), break; end
            moves = rand(1, n) < p;  kb = sum(moves);      % simulate its whole path
            x = 0;
            for r = 1:n
                x = x + (2*moves(r)-1)*0.5*pegGap;  slotPX(slot, r) = x;
            end
            [xr, yr] = settlePos(bx(kb+1), cnt(kb+1), pegGap, ballsAcross, ballD, layerH, floorY);
            cnt(kb+1)   = cnt(kb+1) + 1;
            slotXr(slot) = xr;  slotYr(slot) = yr;
            slotBusy(slot) = true;  slotS(slot) = 0;
            set(movers(slot), 'Visible', 'on');
            moveOnly(movers(slot), (rand-0.5)*0.08, releaseY, ballD);   % enter at the top
            launched = launched + 1;
        end

        % ---- advance every in-flight ball by one step ----
        for slot = find(slotBusy)
            slotS(slot) = slotS(slot) + 1;  s = slotS(slot);
            if s <= n                                   % bouncing through the pegs
                moveOnly(movers(slot), slotPX(slot, s), pegBaseY + (n-s)*pegRowH, ballD);
            elseif s <= n + settleSteps                 % dropping into the heap
                f  = (s - n) / settleSteps;
                xf = slotPX(slot, n) + (slotXr(slot) - slotPX(slot, n))*f;
                yf = pegBaseY + (slotYr(slot) - pegBaseY)*f;
                moveOnly(movers(slot), xf, yf, ballD);
            else                                        % landed: leave a settled ball
                drawBall(ax, slotXr(slot), slotYr(slot), ballD);
                set(movers(slot), 'Visible', 'off');
                slotBusy(slot) = false;  settled = settled + 1;
            end
        end

        drawnow;
        if frameDelay > 0, pause(frameDelay); end
        if makeGif && mod(frame, gifEvery) == 0
            captureGif(hFig, gifFile, gifDelay, false);
        end
    end
else
    % no animation: pour every ball straight into the heap
    for b = 1:N   %#ok<UNRCH>   % reachable when animate = false
        moves = rand(1, n) < p;  kb = sum(moves);
        [xr, yr] = settlePos(bx(kb+1), cnt(kb+1), pegGap, ballsAcross, ballD, layerH, floorY);
        cnt(kb+1) = cnt(kb+1) + 1;
        drawBall(ax, xr, yr, ballD);
        if makeGif && (mod(b, gifEvery*ballsAcross) == 0 || b == N)
            drawnow; captureGif(hFig, gifFile, gifDelay, false);
        end
    end
end

for i = 1:poolSize
    if ishghandle(movers(i)), set(movers(i), 'Visible', 'off'); end
end

% -------------------- optional theoretical overlay ----------------------
if showNormal
    mu    = n*p - n/2;                       % mean in bin-centre (x) coordinates
    sigma = sqrt(n*p*(1-p));
    xg    = linspace(bx(1)-0.5, bx(end)+0.5, 400);
    gpdf  = exp(-(xg-mu).^2 ./ (2*sigma^2)) ./ (sigma*sqrt(2*pi));
    yg    = floorY + (N.*gpdf ./ ballsAcross) .* layerH;   % expected heap height
    plot(ax, xg, yg, 'r-', 'LineWidth', 2);
    text(ax, bx(end)+0.3, max(yg)*0.9, 'Normal', 'Color','r', ...
         'FontWeight','bold','Rotation',0);
end

if makeGif                          % last frame (with the curve), held a bit longer
    drawnow; captureGif(hFig, gifFile, 1.5, false);
    fprintf('  GIF saved   : %s\n', fullfile(pwd, gifFile));
end

% ------------------------------ summary ---------------------------------
sMean = sum(kk.*cnt)/N;
sStd  = sqrt(sum(((kk-sMean).^2).*cnt)/N);
fprintf('\n--- Galton board simulation ---\n');
fprintf('  rows  n     = %d\n', n);
fprintf('  balls N     = %d\n', N);
fprintf('  p(right)    = %.3f\n', p);
fprintf('  sample mean = %7.3f   (theory n*p          = %7.3f)\n', sMean, n*p);
fprintf('  sample std  = %7.3f   (theory sqrt(np(1-p)) = %7.3f)\n', sStd, sqrt(n*p*(1-p)));
fprintf('-------------------------------\n');
end

% ======================= local helper functions =========================
function captureGif(hFig, gifFile, delay, firstFlag)
% Grab the current figure and append it as a frame to the animated GIF.
im = frame2im(getframe(hFig));
[A, map] = rgb2ind(im, 256);
if firstFlag
    imwrite(A, map, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', delay);
else
    imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
end
end

function moveOnly(h, x, y, d)
% Reposition a ball WITHOUT forcing a redraw (one drawnow per frame is enough).
set(h, 'Position', [x-d/2, y-d/2, d, d]);
end

function [xr, yr] = settlePos(bxk, m, pegGap, ballsAcross, ballD, layerH, floorY)
% Resting (x,y) of the m-th ball (0-based) in the bin centred at bxk: a 2-D
% pile, ballsAcross wide, brick-laid layer by layer, with a little jitter.
layer = floor(m / ballsAcross);
col   = mod(m, ballsAcross);
xr = bxk - pegGap/2 + (col + 0.5)*(pegGap/ballsAcross);
if mod(layer,2) == 1, xr = xr + 0.4*(pegGap/ballsAcross); end
xr = xr + (rand-0.5)*0.10*ballD;
yr = floorY + ballD/2 + layer*layerH + (rand-0.5)*0.06*ballD;
xr = min(max(xr, bxk - pegGap/2 + ballD/2), bxk + pegGap/2 - ballD/2);
end

function drawBall(ax, x, y, d)
% Draw one deposited ball as a filled, data-scaled BLUE circle (slight shading).
face = min(max([0.15 0.40 0.85] + (rand-0.5)*0.12, 0), 1);
rectangle(ax, 'Position',[x-d/2, y-d/2, d, d], 'Curvature',[1 1], ...
          'FaceColor',face, 'EdgeColor',[0.10 0.20 0.45], 'LineWidth',0.5);
end

function drawPegs(ax, n, pegGap, pegBaseY, pegRowH)
% Triangular peg array: row r (r=1..n) has r pegs; row 1 on top, row n bottom.
for r = 1:n
    xs = ((1:r) - (r+1)/2) * pegGap;
    ys = (pegBaseY + (n-r)*pegRowH) * ones(1, r);
    plot(ax, xs, ys, 'o', 'MarkerSize',5, ...
         'MarkerFaceColor','w', 'MarkerEdgeColor',[0.35 0.35 0.35], 'LineWidth',1);
end
end

function drawBins(ax, xEdges, floorY, binTopY)
% Vertical bin separators plus the floor line.
for x = xEdges
    plot(ax, [x x], [floorY, binTopY], '-', 'Color',[0.7 0.7 0.7], 'LineWidth',1);
end
plot(ax, [xEdges(1) xEdges(end)], [floorY floorY], '-', ...
     'Color',[0.5 0.5 0.5], 'LineWidth',1.5);
end
