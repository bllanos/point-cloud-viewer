function [phi, beta] = uvToPhiBeta(u, v)
[phi, s] = uToPhi(u);
beta = vToBeta(v);
filter = beta <= 0;
beta(s & filter) = beta(s & filter) + pi;
filter = ~filter;
beta(s & filter) = beta(s & filter) - pi;
end