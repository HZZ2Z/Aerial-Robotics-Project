function ref_sfun(block)
%REF_SFUN Reference mission generator: y = [p*; v*; a*; psi*; psidot*; psiddot*] (15).
setup(block);
end

function setup(block)
block.NumDialogPrms = 1;
block.NumInputPorts  = 0;
block.NumOutputPorts = 1;
block.OutputPort(1).Dimensions = 12;
block.OutputPort(1).SamplingMode = 'Sample';
block.NumContStates = 0;
block.SampleTime = [0, 0];
block.RegBlockMethod('Outputs', @Outputs);
end

function Outputs(block)
P = block.DialogPrm(1).Data;
[pref, vref, aref, psi, psid, psidd] = gen_ref(block.CurrentTime, P);
block.OutputPort(1).Data = [pref; vref; aref; psi; psid; psidd];
end
