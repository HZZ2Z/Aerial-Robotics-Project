%TEST_MODEL Wiring + layout audit for the two Simulink models.
% Wiring: every block port must be connected (dangling in/out ports listed);
% Layout: block bounding boxes must not overlap (functional-zone layout from
% build_closed/build_plant defines positions — this verifies the built model).
% Run after any build_closed/build_plant rebuild; exits nonzero on findings.
function test_model()
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'simulink'));
mdl = {'closed_loop', 'plant_test'};
fail = false;
for k = 1:2
    m = mdl{k};
    fprintf('\n===== %s =====\n', m);
    if ~bdIsLoaded(m), load_system(m); end
    blks = find_system(m, 'LookUnderMasks', 'all', 'Type', 'Block');
    blks = setdiff(blks, find_system(m, 'LookUnderMasks', 'all', ...
        'Type', 'Block', 'BlockType', {'SubSystem'}), 'stable');
    fprintf('blocks: %d\n', numel(blks));

    % --- wiring: dangling ports ---
    dang = {};
    for b = 1:numel(blks)
        ph = get_param(blks{b}, 'PortHandles');
        for p = 1:numel(ph.Inport)
            if get_param(ph.Inport(p), 'Line') == -1
                dang{end+1} = sprintf('IN  %s/%d', blks{b}, p); %#ok<AGROW>
            end
        end
        for p = 1:numel(ph.Outport)
            if get_param(ph.Outport(p), 'Line') == -1
                dang{end+1} = sprintf('OUT %s/%d', blks{b}, p); %#ok<AGROW>
            end
        end
    end
    if isempty(dang)
        fprintf('wiring: OK (no dangling ports)\n');
    else
        fprintf('wiring: %d DANGLING PORTS:\n', numel(dang));
        for d = 1:numel(dang), fprintf('  %s\n', dang{d}); end
        fail = true;
    end

    % --- layout: overlapping bounding boxes ---
    pos = {};
    for b = 1:numel(blks)
        try
            p = get_param(blks{b}, 'Position');
            pos{end+1} = struct('name', blks{b}, 'r', p); %#ok<AGROW>
        catch
            % ports/annotations without Position — skip
        end
    end
    ov = {};
    for a = 1:numel(pos)
        for c = a+1:numel(pos)
            if boxes_overlap(pos{a}.r, pos{c}.r)
                ov{end+1} = sprintf('%s  <->  %s', pos{a}.name, pos{c}.name); %#ok<AGROW>
            end
        end
    end
    if isempty(ov)
        fprintf('layout: OK (no block overlaps)\n');
    else
        fprintf('layout: %d OVERLAPPING PAIRS:\n', numel(ov));
        for d = 1:numel(ov), fprintf('  %s\n', ov{d}); end
        fail = true;
    end
    close_system(m, 0);
end
if fail, error('test_model: findings above'); end
fprintf('\nMODEL AUDIT PASSED\n');
end

function tf = boxes_overlap(a, b)
% strict interior overlap (touching edges are fine)
tf = a(1) < b(3) && b(1) < a(3) && a(2) < b(4) && b(2) < a(4);
end
