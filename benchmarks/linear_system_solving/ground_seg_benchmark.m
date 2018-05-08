filename = 'linear_system_benchmark_1.csv';
% filename = 'linear_system_benchmark_2.csv';

raw_values = csvread(filename);

multiplier = 0.5;
t_min = 0.0;
t_max = 14.103877 * multiplier;
x_max = t_max + 1.0
max_value = 50;
valid_mask = raw_values < max_value;

valid_data = raw_values(valid_mask, 1);
num_values = floor(length(valid_data) * multiplier)
chosen_data = valid_data(1:num_values);

t = linspace(t_min, t_max, num_values);
data_mean = mean(chosen_data, 1);
data_std = std(chosen_data, 1);

figure
hold on
plot(t, chosen_data)
plot(t, ones(num_values,1) * data_mean, 'r*');
plot(t, ones(num_values,1) * (data_mean+data_std), 'g*');
plot(t, ones(num_values,1) * (data_mean-data_std), 'g*');

mean_text = ' \leftarrow mean = ' + string(data_mean);
text(t(end), data_mean, mean_text)

title('Ratio of matrix/qrpiv elapsed time')
xlabel('Time (s)')
ylabel('Ratio')
xlim([0 x_max])
legend('data', 'mean', 'std dev')

