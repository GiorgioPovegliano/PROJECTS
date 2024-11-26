classdef audio_utils
    methods (Static)

        function [y, t] = get_audio(structure_with_time, plot_result)

            buffer = structure_with_time.signals.values;
            last_time = structure_with_time.time(end,1);
            t = linspace(0,last_time, size(buffer,1)*size(structure_with_time.time,1));
        
            l = size(buffer, 3);
            y = [];
            for i = 1:l
                y = cat(1, y, buffer(:,1,i)); 
            end
            if plot_result == 1
                xlabel('time elapsed [s]') 
                ylabel('Amplitude') 
                plot(t,y)
            end
        end
    end % static methods
end % classdef