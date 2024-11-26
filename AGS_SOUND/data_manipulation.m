D = readtable('data.csv');
mean_age = mean(D{:,2});
license = sum(D{:,3} == 1);
a1 = sum(D{:,4} == 1);
a2 = sum(D{:,4} == 2);
a3 = sum(D{:,4} == 3);
a4 = sum(D{:,4} == 4);
a5 = sum(D{:,4} == 5);

% normalization of the data
D{:, 5:31} = D{:, 5:31} / 100;

aggressive_data = [];
refined_data = [];
futuristic_data = [];

mean_agg = zeros(9, 1);
std_agg = zeros(9, 1);
mean_ref = zeros(9, 1);
std_ref = zeros(9, 1);
mean_fut = zeros(9, 1);
std_fut = zeros(9, 1);

group_index = 1;

for i = 5:3:29
    aggressive_data = [aggressive_data; D{:,i}];        
    refined_data = [refined_data; D{:,i+1}];           
    futuristic_data = [futuristic_data; D{:,i+2}];     
    
    mean_agg(group_index) = mean(D{:,i});
    std_agg(group_index) = std(D{:,i});
    
    mean_ref(group_index) = mean(D{:,i+1});
    std_ref(group_index) = std(D{:,i+1});
    
    mean_fut(group_index) = mean(D{:,i+2});
    std_fut(group_index) = std(D{:,i+2});
    
    group_index = group_index + 1;
end

num_rows_per_group = size(D, 1);
group = repelem(1:9, num_rows_per_group)';

% ANOVA
[p_agg, tbl_agg, stats_agg] = anova1(aggressive_data, group, 'on');
[p_ref, tbl_ref, stats_ref] = anova1(refined_data, group, 'on');
[p_fut, tbl_fut, stats_fut] = anova1(futuristic_data, group, 'on');

% KRUSKALWALLIS
%[p_agg, tbl_agg, stats_agg] = kruskalwallis(aggressive_data,group, 'on');
%[p_ref, tbl_ref, stats_ref] = kruskalwallis(refined_data, group, 'on');
%[p_fut, tbl_fut, stats_fut] = kruskalwallis(futuristic_data, group, 'on');

%disp('Aggressive ANOVA Results:');
%disp(tbl_agg);

%disp('Refined ANOVA Results:');
%disp(tbl_ref);

%disp('Futuristic ANOVA Results:');
%disp(tbl_fut);

comparison_agg = multcompare(stats_agg);
%comparison_ref = multcompare(stats_ref);
%comparison_fut = multcompare(stats_fut);

disp('Mean ± Standard Deviation for Each Sound:');

for sound = 1:9
    fprintf('Sound %d - Aggressive: %.2f ± %.2f\n', sound, mean_agg(sound), std_agg(sound));
    fprintf('Sound %d - Refined: %.2f ± %.2f\n', sound, mean_ref(sound), std_ref(sound));
    fprintf('Sound %d - Futuristic: %.2f ± %.2f\n', sound, mean_fut(sound), std_fut(sound));
end

%aggressive_data_reshaped = [aggressive_data,group];
%welchanova(aggressive_data_reshaped);
