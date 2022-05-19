function model = genSerialLink(nb)

%% Params
syms m Lc L Ic [nb 1] real
L = [0; L(1:end-1)];

%% Model Construct
model.NB = nb;
model.parent = 0:1:(nb-1);

for i=1:nb
    model.jtype{i} = 'R';         % joint type
    model.com{i}   = [Lc(i) 0 0]';
    model.mass{i} = m(i);
    model.I{i} = mcI(model.mass{i}, model.com{i}, diag([0, 0, Ic(i)]));
    model.Xtree{i} = plux(eye(3), [L(i) 0 0]');
end
end