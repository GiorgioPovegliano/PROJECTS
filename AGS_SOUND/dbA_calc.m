[y,Fs1] = audioread('futuristic_A.wav');
[x,Fs2] = audioread('futuristic.wav');
RMS_signal_dBA = 20*log10(rms(y)/1)
RMS_signal_dB = 20*log10(rms(x)/1)