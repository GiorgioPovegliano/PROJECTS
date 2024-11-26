if exist('fs','var')
    if fs ~= f_resample
        audio_1 = resample(audio_1, f_resample, fs);
        fs = f_resample;
    end
    max_s1 = length(audio_1);
end

if exist('fs2','var')
    if fs2 ~= f_resample
        audio_2 = resample(audio_2, f_resample, fs2);
        fs2 = f_resample;
    end
    max_s2 = length(audio_2);
end

if exist('fs3','var')
    if fs ~= fs3
        audio_3 = resample(audio_3, f_resample, fs3);
        fs3 = f_resample;
    end
    max_s3 = length(audio_3);
end


