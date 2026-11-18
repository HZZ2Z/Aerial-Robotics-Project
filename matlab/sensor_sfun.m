function sensor_sfun(block)
%SENSOR_SFUN Legacy sensor extension: sample & hold of true signal + noise.
% NOT used by the course baseline: build_closed wires the course model
% (continuous true signal + band-limited white noise of density r) natively.
% Kept as a clearly marked engineering extension (per-instance seeds).
% in1 = true signal (3), in2 = fix status (1, used only for gnss mode);
% out = measured (3).
% mode: 1=AHRS (200 Hz), 2=rate gyro (200 Hz), 3=GNSS (50 Hz, fix-switched)
setup(block);
end

function setup(block)
P  = block.DialogPrm(1).Data;
md = block.DialogPrm(2).Data;
block.NumDialogPrms = 2;
block.NumInputPorts  = 2;
block.NumOutputPorts = 1;
block.InputPort(1).Dimensions  = 3;
block.InputPort(2).Dimensions  = 1;
block.OutputPort(1).Dimensions = 3;
block.OutputPort(1).SamplingMode = 'Sample';
% FT declared so inputs are valid in Update; the output is still a
% sample-and-hold value, so no algebraic loop forms.
block.InputPort(1).DirectFeedthrough  = true;
block.InputPort(2).DirectFeedthrough  = true;
switch md
    case 1, Ts = P.sens.ahrs.rate^-1;
    case 2, Ts = P.sens.gyro.rate^-1;
    case 3, Ts = P.sens.gnss.rate^-1;
end
block.SampleTime = [Ts, 0];
block.NumContStates = 0;
block.RegBlockMethod('Start',   @StartFcn);
block.RegBlockMethod('Outputs', @Outputs);
block.RegBlockMethod('Update',  @Update);
end

function StartFcn(block)
md = block.DialogPrm(2).Data;
hold_state(md, zeros(3,1));   % fresh hold and noise stream every simulation
noise_stream(md, true);
end

function Update(block)
P  = block.DialogPrm(1).Data;
md = block.DialogPrm(2).Data;
rs = noise_stream(md);
x = block.InputPort(1).Data;
switch md
    case 1, sig = P.sens.ahrs.sigma*ones(3,1);
    case 2, sig = P.sens.gyro.sigma*ones(3,1);
    case 3
        sig = P.sens.gnss.sigma;
        if block.InputPort(2).Data < 0.5     % RTK fix lost: degraded sigma
            sig = P.sens.gnss.bad;
        end
end
hold_state(md, x + sig .* randn(rs, [3 1]));   % sample & hold
end

function Outputs(block)
md = block.DialogPrm(2).Data;
block.OutputPort(1).Data = hold_state(md);
end

function y = hold_state(md, xnew)
% shared sample-and-hold storage (persistent is per-function in MATLAB,
% so both Update and Outputs go through this accessor)
persistent yh
if isempty(yh), yh = cell(1,3); end
if isempty(yh{md}), yh{md} = zeros(3,1); end
if nargin > 1, yh{md} = xnew; end
y = yh{md};
end

function st = noise_stream(md, reseed)
persistent streams
if isempty(streams), streams = cell(1,3); end
if nargin > 1 && reseed, streams{md} = RandStream('twister', 'Seed', 100 + md); end
if isempty(streams{md}), streams{md} = RandStream('twister', 'Seed', 100 + md); end
st = streams{md};
end
