function Nu = nu(lam)
Nu = cell(1,length(lam));
for i = length(lam):-1:1
    Nu{i} = [i Nu{i}];
    if lam(i) ~= 0
        Nu{lam(i)} = [Nu{i} Nu{lam(i)}];
    end
end
end
