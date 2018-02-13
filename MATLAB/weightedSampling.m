function [ filter ] = weightedSampling( weights, count )
% WEIGHTEDSAMPLING  Sample according to probability weights
%
% ## Syntax
% filter = weightedSampling( weights, count )
%
% ## Description
% filter = weightedSampling( weights, count )
%   Returns sampling indices corresponding to the input weights vector
%
% ## Input Arguments
%
% weights -- Element importance weights
%   A column vector containing weights for a set of elements.
%
% count -- Sample size
%   The number of elements to sample. `count` must be at most
%   `length(weights)`
%
% ## Output Arguments
%
% filter -- Sampling indices
%   A column vector of length `count`, containing indices of the elements
%   in `weights` to be sampled. The i-th element of `weights` will be
%   chosen with probability `(weights(i) / max(weights)) * (count /
%   length(weights)`. The indices in `filter` are in order.

% Created for: CMPUT 511 Project
% Fall 2017
% Bernard Llanos
% Department of Computing Science, University of Alberta

nargoutchk(1, 1);
narginchk(2, 2);

n = length(weights);
if count >= n
    error('`count` must be less than the length of `weights`.')
end

weights = weights / max(weights);
pass = rand(n, 1) < weights;
first = find(pass);
first = first(randperm(length(first)));
second = find(~pass);
second = second(randperm(length(second)));
all = [first; second];
filter = all(1:count);
filter = sort(filter);
end
