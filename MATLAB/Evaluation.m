%% Normal Estimation Evaluation
% Visualize the CSV output file from the point cloud renderer, and
% calculate the error for different normal estimation algorithms.
%
% ## Usage
%   Modify the parameters, then run.
%
% Created for: CMPUT 511 Project
% Fall 2017
% Bernard Llanos
% Department of Computing Science, University of Alberta

%% Parameters

% Input file
input_file = '../results.csv';
data = csvread(input_file);

% Evaluation parameters
threshold_bad = 10 * pi / 180; % Bad normal vectors have an error of >= 10 degrees
threshold_very_bad = 45 * pi / 180; % Very bad normal vectors have an error of >= 45 degrees

% ## Output Parameters
plotting_enabled = true; % Plot tangent vector estimates

%% Visually assess tangent vectors
dimension = 3;

curve_indices = reshape(data(1, :), dimension, []).';
curve_indices = curve_indices(:, 1);
curve_counts = diff([0; find(diff(curve_indices)); length(curve_indices)]);
x = reshape(data(2, :), dimension, []).';
x_cell = mat2cell(x, curve_counts, dimension);
n_curves = size(x_cell, 1);
n = reshape(data(4, :), dimension, []).';
n_cell = mat2cell(n, curve_counts, dimension);
t = reshape(data(3, :), dimension, []).';
t_cell = mat2cell(t, curve_counts, dimension);

if plotting_enabled
    figure;
    hold on
    for c = 1:n_curves
        color = rand(3, 1);
        %plot3(x_cell{c}(:, 1), x_cell{c}(:, 2), x_cell{c}(:, 3), 'Color', color);
        %quiver3(x_cell{c}(:, 1), x_cell{c}(:, 2), x_cell{c}(:, 3), n_cell{c}(:, 1), n_cell{c}(:, 2), n_cell{c}(:, 3), 'Color', color);
        quiver3(x_cell{c}(:, 1), x_cell{c}(:, 2), x_cell{c}(:, 3), t_cell{c}(:, 1), t_cell{c}(:, 2), t_cell{c}(:, 3), 'Color', color / 2);
    end
    title('Estimated tangent vectors')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    axis equal
    hold off
end

%% Calculate aggregated normal estimation error metrics

normal_estimation_methods = {
    'Delaunay balls', 'Delaunay balls & Tangent', 'VCM', 'VCM & Tangent', 'PCA', 'PCA & Tangent'
    }.';

n_normal_estimation_methods = length(normal_estimation_methods);
rmse = zeros(n_normal_estimation_methods, 1);
n_bad_points = zeros(n_normal_estimation_methods, 1);
n_very_bad_points = zeros(n_normal_estimation_methods, 1);
n_points = size(x, 1);
for i = 1:n_normal_estimation_methods
    n_est = reshape(data(4 + i, :), dimension, []).';
    angle = acos(max(min(dot(n, n_est, 2), 1), -1));
    rmse(i) = sqrt(sum(angle.^2) / n_points) * 180 / pi;
    n_bad_points(i) = sum(angle > threshold_bad) / n_points * 100;
    n_very_bad_points(i) = sum(angle > threshold_very_bad) / n_points * 100;
end

results = table(...
    normal_estimation_methods,...
    rmse,...
    n_bad_points,...
    n_very_bad_points,...
    'VariableNames', {...
        'Algorithm', 'RMSE',...
        'Bad',...
        'VeryBad'...
        }...
    );
disp(results);
