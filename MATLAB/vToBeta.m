function beta = vToBeta(v)
v = rem(v, 1);
v(v < 0) = 1 + v(v < 0);
beta = (v * (2 * pi)) - pi;
end