%% »≈∂Ø”Î‘Î…˘…Ë÷√

function [ Signals ] = PerNoise( )

deg = pi/180;
Signals.PerbSigma = [0.1;0.1; 0.1;1*deg;1*deg;1*deg ];
Signals.PerbMu    = ones(6,1)*0;
Signals.PerbVal   =  [ 1;1;1;1;1;1]*100*1;
   
Signals.NoiseSigma = [ 0.01;0.01;0.01;1*deg;1*deg;1*deg];
Signals.NoiseMu    = ones(6,1)*0;
Signals.NoiseVal   = [ 1;1;1;1;1; 1 ]*1*1;%  6*deg;6*deg;6*deg

end

