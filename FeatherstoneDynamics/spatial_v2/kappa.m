function Ka = kappa(lam)

n = length(lam);
Ka = cell(1, n);
for i=1:n
    if lam(i) == 0
        Ka{i} = i;
    else
        Ka{i} = [Ka{lam(i)},i];
    end
end


end

