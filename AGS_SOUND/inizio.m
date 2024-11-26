clc
%clear all

% samples per frame
spf_mat = 1024;

% max grain size in frames
%max_grain_size = 25000;
% min grain size in frames
%min_grain_size = 8000;
% variance on the grains creation
%max_variance = 4000;

rng_seeds_thread_1 = [1,2,3,4,5,6,7,8];
rng_seeds_thread_2 = [9,10,11,12,13,14,15,16];
rng_seeds_thread_3 = [17,18,19,20,21,22,23,24];

if isfile('default_audio.mat')
    load('default_audio.mat')
else
    load('backup_audio.mat')
end
% Detect OS
if ismac || isunix
    selected_audio.filepath1 = strrep(selected_audio.filepath1,'\','/');
    selected_audio.filepath2 = strrep(selected_audio.filepath2,'\','/');
    selected_audio.filepath3 = strrep(selected_audio.filepath3,'\','/');
end
[audio_1_nr,fs] = audioread(selected_audio.filepath1);
[audio_2_nr,fs2] = audioread(selected_audio.filepath2);
[audio_3_nr,fs3] = audioread(selected_audio.filepath3);

if fs ~= f_resample
    audio_1 = resample(audio_1_nr, f_resample, fs);
    fs = f_resample;
else
    audio_1 = audio_2_nr;
end

if fs2 ~= f_resample
    audio_2 = resample(audio_2_nr, f_resample, fs2);
    fs2 = f_resample;
else
    audio_2 = audio_2_nr;
end

if fs3 ~= f_resample
    audio_3 = resample(audio_3_nr, f_resample, fs3);
    fs3 = f_resample;
else
    audio_3 = audio_3_nr;
end

% max samples
%max_s = 110250;

max_s1 = length(audio_1);
max_s2 = length(audio_2);
max_s3 = length(audio_3);

