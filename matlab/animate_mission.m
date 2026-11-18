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
hv = plot3(ax, NaN, NaN, NaN, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6);
he = plot3(ax, NaN, NaN, NaN, 'g-',  'LineWidth', 1.2);
legend(ax, {'reference', 'true', 'estimated', 'vehicle', '1\sigma ellipse (P_{xy} eig)'}, ...
    'Location', 'best');
axis(ax, 'equal');

th = linspace(0, 2*pi, 40);
v = VideoWriter(fullfile(root, 'results', fname), 'MPEG-4');
v.FrameRate = 25; open(v);
n = numel(o.t); step = max(1, round(n/500));
for k = 1:step:n
    set(hT, 'XData', o.p(1:k,1),  'YData', o.p(1:k,2),  'ZData', o.p(1:k,3));
    set(hE, 'XData', o.kf(1:k,1), 'YData', o.kf(1:k,2), 'ZData', o.kf(1:k,3));
    set(hv, 'XData', o.p(k,1), 'YData', o.p(k,2), 'ZData', o.p(k,3));
    Pxy = o.P_out(k, 1:2, 1:2);  Pxy = (Pxy + Pxy')/2;
    [V, D] = eig(Pxy);
    E = o.kf(k,1:2)' + V*diag(sqrt(max(diag(D), 0)))*[cos(th); sin(th)];
    set(he, 'XData', E(1,:), 'YData', E(2,:), 'ZData', o.kf(k,3) + 0*th);
    drawnow;
    writeVideo(v, getframe(fig));
end
close(v); close(fig);
anim = fullfile(root, 'results', fname);
fprintf('animation saved: %s\n', anim);
end
