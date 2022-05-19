function Mu = mu(lam)

n = length(lam);
for i=1:n
    Mu{i} = find((i-1)==lam);
end

end