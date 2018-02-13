%% Curve-based sampling of superellipsoids
% Trace curves along the surfaces of superellipsoids, and output the
% sampled points and normal vectors to CSV files
%
% ## Usage
%   Modify the parameters, then run.
%
% ## References
% - Paul Bourke's webpage, "Superellipse and Superellipsoid: A Geometric
%   Primitive for Computer Aided Design" (January 1990), at
%   http://paulbourke.net/geometry/superellipse/
%
% Created for: CMPUT 511 Project
% Fall 2017
% Bernard Llanos
% Department of Computing Science, University of Alberta

%% Parameters

% ## Superellipsoid parameters
% See http://paulbourke.net/geometry/superellipse/ for parameter
% descriptions
r = [1, 1, 1]; % Radii in the x, y, and z dimensions
n1 = 1; % Curvature in the z dimension
n2 = 3; % Curvature in the x and y dimensions

% ## Curve parameters
curve_length = 0.5; % Length of curves in the point cloud, as a fraction of 2 * pi in surface parameter space
axis_aligned = false; % Whether or not to force curves to correspond to axis-aligned lines in parameter space
include_corners = false; % Whether or not to include the "corner curves" (the equator, and the 0, 90, 180, and 270 degrees meridians)

% ## Sampling parameters
n_curves = 50; % Number of curves to produce
n_points_per_curve = 50; % Number of points to sample on each curve
% Standard deviation of Gaussian noise. Noise is a vector offset sampled
% uniformly from the possible 3D directions, and with a length drawn from a
% normal distribution with a mean of zero and a standard deviation equal to
% `noise(k)` times the average spacing between points on the curves.
noise = [0, 0.01, 0.05, 0.25, 0.5];

% ## Output Parameters
plotting_enabled = true; % Visualize the point clouds instead of just saving them to a file
output_directory = '../superellipsoids'; % Where to save the point clouds

%% Processing

for i = 1:length(include_corners)
    for j = 1:length(axis_aligned)
        for k = 1:size(n1, 1)
            for m = 1:size(n2, 1)
                [ x_true, n, t ] = superellipsoidCurveSampler(...
                    r, n1(k), n2(m), curve_length, axis_aligned(j),...
                    include_corners(i), n_curves, n_points_per_curve...
                    );
                for no = 1:length(noise)
                    x = corrupt(x_true, noise(no));
                    if plotting_enabled
                        figure;
                        hold on
                        for c = 1:n_curves
                            color = rand(3, 1);
                            plot3(x{c}(:, 1), x{c}(:, 2), x{c}(:, 3), 'Color', color);
                            %quiver3(x{c}(:, 1), x{c}(:, 2), x{c}(:, 3), n{c}(:, 1), n{c}(:, 2), n{c}(:, 3), 'Color', color);
                            %quiver3(x{c}(:, 1), x{c}(:, 2), x{c}(:, 3), t{c}(:, 1), t{c}(:, 2), t{c}(:, 3), 'Color', color / 2);
                        end
                        title(sprintf(...
                            'Superellipsoid r = (%g, %g, %g), n_1 = %g, n_2 = %g, noise = %g',...
                            r(1), r(2), r(3), n1(k), n2(m), noise(no)...
                            ))
                        xlabel('X')
                        ylabel('Y')
                        zlabel('Z')
                        xlim([-r(1), r(1)])
                        ylim([-r(2), r(2)])
                        zlim([-r(3), r(3)])
                        axis equal
                        hold off
                    end
                    name = sprintf('r_%g_%g_%g_n1_%g_n2_%g_len_%g', r(1), r(2), r(3), n1(k), n2(m), curve_length);
                    if axis_aligned
                        name = [name, '_axisAligned'];
                    end
                    if include_corners
                        name = [name, '_withCorners'];
                    end
                    name = [name,...
                        sprintf('_%d_curves_%d_ppc_%g_noise.csv',...
                            n_curves, n_points_per_curve, noise(no)...
                        )]; %#ok<AGROW>
                    saveCurves( fullfile(output_directory, name), x, n, t );
                end
            end
        end
    end
end
