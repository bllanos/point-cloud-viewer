% `s` is a boolean vector indicating (where `true`) if the location is
% past the north or south pole, in which case, `beta` must be
% translated by `pi`.
function [phi, s] = uToPhi(u)
s = floor(u);
s = logical(mod(s, 2));
phi = rem(u, 1);
phi(phi < 0) = 1 + phi(phi < 0);
phi(s) = 1 - phi(s);
phi = (phi * pi) - (pi / 2);
end