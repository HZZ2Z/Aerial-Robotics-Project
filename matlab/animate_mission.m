function anim = animate_mission(o, P, fname)
%ANIMATE_MISSION 3-D animation: vehicle, reference, true and estimated
% trajectories, horizontal 1-sigma covariance ellipse (eig of P9(1:2,1:2),
% covariance orientation preserved; axis lengths are 1-sigma, NOT 68%/95%).
% Saves mp4. All data comes from the single run o (including P_out).
root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 3 || isempty(fname), fname = 'mission_anim.mp4'; end

fig = figure('Color', 'w', 'Position', [80 80 900 650]);
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); view(ax, 40, 22);
xlabel(ax, 'x North [m]'); ylabel(ax, 'y East [m]'); zlabel(ax, 'z Down [m]');
plot3(ax, o.ref(:,1), o.ref(:,2), o.ref(:,3), 'k--', 'LineWidth', 1);
hT = plot3(ax, NaN, NaN, NaN, 'b-',  'LineWidth', 1.2);
hE = plot3(ax, NaN, NaN, NaN, 'r-.', 'LineWidth', 1);
hv = plot3(ax, NaN, NaN, NaN, 'o-', 'Color', [0.1 0.35 0.55], ...
    'MarkerFaceColor', [0.1 0.35 0.55], 'LineWidth', 2, 'MarkerSize', 3);
heading = plot3(ax, NaN, NaN, NaN, '-', 'Color', [0.94 0.43 0.12], ...
    'LineWidth', 2, 'HandleVisibility', 'off');
he = plot3(ax, NaN, NaN, NaN, 'g-',  'LineWidth', 1.2);
legend(ax, {'reference', 'true', 'estimated', 'vehicle', '1\sigma ellipse (P_{xy} eig)'}, ...
    'Location', 'best');
axis(ax, 'equal');

th = linspace(0, 2*pi, 40);
v = VideoWriter(fullfile(root, 'results', fname), 'MPEG-4');
v.FrameRate = 25; open(v);
n = numel(o.t);
frames = unique(round(linspace(1,n,402)));
span = 0.23;
body = span*[-1 1 NaN 0 0; 0 0 NaN -1 1; 0 0 NaN 0 0];
for k = frames
    set(hT, 'XData', o.p(1:k,1),  'YData', o.p(1:k,2),  'ZData', o.p(1:k,3));
    set(hE, 'XData', o.kf(1:k,1), 'YData', o.kf(1:k,2), 'ZData', o.kf(1:k,3));
    R = reshape(o.R(k,:),3,3);
    arms = o.p(k,:)' + R*body;
    nose = o.p(k,:)' + R*[0 1.35*span;0 0;0 0];
    set(hv, 'XData', arms(1,:), 'YData', arms(2,:), 'ZData', arms(3,:));
    set(heading,'XData',nose(1,:),'YData',nose(2,:),'ZData',nose(3,:));
    Pxy = squeeze(o.P_out(k, 1:2, 1:2));  Pxy = (Pxy + Pxy')/2;
    [V, D] = eig(Pxy);
    E = o.kf(k,1:2)' + V*diag(sqrt(max(diag(D), 0)))*[cos(th); sin(th)];
    set(he, 'XData', E(1,:), 'YData', E(2,:), 'ZData', o.kf(k,3) + 0*th);
    title(ax,sprintf('Mission time %.1f s',o.t(k)));
    drawnow;
    writeVideo(v, getframe(fig));
end
close(v); close(fig);
anim = fullfile(root, 'results', fname);
fprintf('animation saved: %s\n', anim);
end
