function [ x, n, t ] = superellipsoidCurveSampler( r, n1, n2, len, axis_aligned, include_corners, count, count_points )
% SUPERELLIPSOIDCURVESAMPLER  Sample a superellipsoid along curves in parameter space
%
% ## Syntax
% x = superellipsoidCurveSampler( r, n1, n2, len, axis_aligned, include_corners, count, count_points )
% [ x, n ] = superellipsoidCurveSampler( r, n1, n2, len, axis_aligned, include_corners, count, count_points )
% [ x, n, t ] = superellipsoidCurveSampler( r, n1, n2, len, axis_aligned, include_corners, count, count_points )
%
% ## Description
% x = superellipsoidCurveSampler( r, n1, n2, len, axis_aligned, include_corners, count, count_points )
%   Returns positions on curves on the surface of the superellipsoid
% [ x, n ] = superellipsoidCurveSampler( r, n1, n2, len, axis_aligned, include_corners, count, count_points )
%   Additionally returns normal vectors at the surface positions
% [ x, n, t ] = superellipsoidCurveSampler( r, n1, n2, len, axis_aligned, include_corners, count, count_points )
%   Additionally returns tangent vectors at the surface positions
%
% ## Input Arguments
%
% r -- Superellipsoid radii
%   A three-element vector giving the radii of the superellipsoid in the x,
%   y, and z dimensions, respectively.
%
% n1 -- First exponent
%   The exponent applied to trigonometric functions of 'phi' in the
%   parametric form of the superellipsoid. A scalar.
%
% n2 -- Second exponent
%   The exponent applied to trigonometric functions of 'beta' in the
%   parametric form of the superellipsoid. A scalar.
%
% len -- Curve length
%   The length of the line in parameter space, expressed in the range (0,
%   1], where 1 corresponds to a length of `2 * pi`.
%
% axis_aligned -- Axis alignment of parameter lines
%   If true, the sampled curves will correspond to lines of constant 'phi'
%   or constant 'beta' in the parameterization of the superellipsoid.
%
%   If false, the sampled curves will correspond to general line through
%   the superellipsoid parameter space.
%
% include_corners -- Include corner curves
%   If `true`, the sampled curves will always begin with the following 5
%   curves. The first four curves completing a half revolution around the
%   superellipsoid, from pole to pole, whereas the fifth completes a full
%   revolution around the equator. Each curve will be repeated once in each
%   direction:
%   - beta = 0
%   - beta = pi / 2
%   - beta = pi
%   - beta = - pi / 2
%   - phi = 0
%
%   Therefore, if `include_corners` is `true`, `count` must be at least 10.
%
% count -- Number of curves
%   The number of curves to output. Curves other than the corner curves
%   (if applicable - if `include_corners` is `true`) are sampled randomly.
%
% count_points -- Number of points per curve
%   The number of points to sample on each curve
%
% ## Output Arguments
%
% x -- Positions
%   A cell column vector of length `count`, where the elements are
%   count_points x 3 arrays. `x{k}(i, :)` is the 3D position of the i-th
%   sample point on the surface of the superellipsoid, in the k-th curve.
%
% n -- Normals
%   A cell column vector of length `count`, where the elements are
%   count_points x 3 arrays. `n{k}(i, :)` is the 3D normal vector at the
%   position `x{k}(i, :)`.
%
% t -- Tangents
%   Analogous to `n`, but contains 3D unit tangent vectors at the curve
%   points.
%
% ## Notes
% - The lines in parameter space are selected by choosing random origin
%   points and random direction vectors. Origin points are chosen such that
%   each area element on the superellipsoid has equal probability.
%
% ## References
% - Paul Bourke's webpage, "Superellipse and Superellipsoid: A Geometric
%   Primitive for Computer Aided Design" (January 1990), at
%   http://paulbourke.net/geometry/superellipse/

% Created for: CMPUT 511 Project
% Fall 2017
% Bernard Llanos
% Department of Computing Science, University of Alberta

nargoutchk(1, 3);
narginchk(8, 8);

n_corners = 10;
if include_corners && count < n_corners
    error('If `include_corners` is `true`, `count` must be at least %d.', n_corners)
end

% To sample points uniformly by area differential, I need to find the range
% of area differentials. I will oversample, to find the range of area
% differentials, then subsample the points.
sampling_factor = 10;
super_count = count * sampling_factor;

p = rand(super_count, 2);
% Sample points on the surface
[super_phi, super_beta] = uvToPhiBeta(p(:, 1), p(:, 2));

[ ~, ~, ~, ~, super_A ] = superellipsoid( super_phi, super_beta, r, n1, n2 );

% Sample points at random, accounting for area differentials
probabilities = super_A .* rand(super_count, 1);
if include_corners
    reduced_count = count - n_corners;
else
    reduced_count = count;
end
filter = weightedSampling(probabilities, reduced_count);
phi = super_phi(filter, :);
beta = super_beta(filter, :);
if axis_aligned
    theta = (randi(4, reduced_count, 1) - 1) * pi / 2;
else
    theta = rand(reduced_count, 1) * 2 * pi;
end
lengths = repmat(len, reduced_count, 1);
if include_corners
    phi = [repmat([0; 0; 0; 0; 0], 2, 1); phi];
    beta = [repmat([0; pi/2; pi; -pi/2; 0], 2, 1); beta];
    theta = [
        0; 0; 0; 0; pi/2;
        pi; pi; pi; pi; 3 * pi / 2;
        theta];
    lengths = [0.5 * ones(4, 1); 1; 0.5 * ones(4, 1); 1; lengths];
end

x = cell(count, 1);
n = cell(count, 1);
t = cell(count, 1);
for i = 1:count
    [ x{i}, n{i}, t{i} ] = superellipsoidCurve(...
        r, n1, n2, phi(i), beta(i), theta(i), lengths(i), count_points...
        );
end

end
