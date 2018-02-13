function x = corrupt( x_in, noise_std )
% CORRUPT  Corrupt curve sample positions with Gaussian noise
%
% ## Syntax
% x = corrupt( x_in, noise_std )
%
% ## Description
% x = corrupt( x_in, noise_std )
%   Returns noisy versions of the input positions
%
% ## Input Arguments
%
% x_in -- Original positions
%   A cell column vector, where the elements are 3-column arrays.
%   `x{k}(i,:)` is the 3D position of the i-th sample point on the k-th
%   curve.
%
% noise_std -- Noise standard deviation
%   Positions in `x_in` are perturbed by noise vectors with lengths drawn
%   from a Gaussian distribution having mean zero, and a standard deviation
%   equal to the average spacing between adjacent points on the curves
%   multiplied by `noise_std`.
%
% ## Output Arguments
%
% x -- Perturbed positions
%   A version of `x_in` where the positions have been perturbed by noise
%   vectors with lengths drawn from a Gaussian distribution having mean
%   zero, and a standard deviation equal to the average spacing between
%   adjacent points on the curves multiplied by `noise_std`. The directions
%   of the noise vectors are sampled uniformly on the unit sphere.
%
% ## References
% - Picking points uniformly on a sphere:
%   http://mathworld.wolfram.com/SpherePointPicking.html

% Created for: CMPUT 511 Project
% Fall 2017
% Bernard Llanos
% Department of Computing Science, University of Alberta

nargoutchk(1, 1);
narginchk(2, 2);

% Find the average spacing between points
n_curves = length(x_in);
n_points_in_curves = zeros(n_curves, 1);
mean_lengths = zeros(n_curves, 1);
for c = 1:n_curves
    segments = diff(x_in{c}, 1, 1);
    lengths = sqrt(dot(segments, segments, 2));
    n_points_in_curves(c) = size(x_in{c}, 1);
    mean_lengths(c) = mean(lengths);
end
average_length = dot(n_points_in_curves - 1, mean_lengths) / sum(n_points_in_curves - 1);

% Add noise to the points
x = cell(n_curves, 1);
for c = 1:n_curves
    uv = rand(n_points_in_curves(c), 2);
    theta_cosphi = [2*pi*uv(:, 1), 2 * uv(:, 2) - 1];
    sinphi = sqrt(1 - theta_cosphi(:, 2) .^ 2);
    offsets = [
        sinphi .* cos(theta_cosphi(:, 1)),...
        sinphi .* sin(theta_cosphi(:, 1)),...
        theta_cosphi(:, 2)
    ];
    lengths = normrnd(0, average_length * noise_std, n_points_in_curves(c), 1);
    offsets = offsets .* repmat(lengths, 1, 3);
    x{c} = x_in{c} + offsets;
end

end
