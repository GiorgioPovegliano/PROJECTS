% To Multimedia Block on Simulink does not support variable size signal, so
% the sound is saved inside a workspace and manually converted to a .wav
A = audio_output.signals.values;    % Normal audio file
%B = audio_output_A.signals.values;  % Audio weighted with A-filter 
A = reshape(A.',1,[]);
%B = reshape(B.',1,[]);
v = A';
%z = B';
audiowrite('Sounds_acc/pulse_acc.wav',v,fs);
%audiowrite('futuristic_A.wav',z,fs);