function floatingModel = genFloatingSerialLink(nbExceptBase)
nb = nbExceptBase + 1;
parent = (1:nb)-1;

%% Params
syms m Lc L I [nbExceptBase 1] real
syms mb Lcb Ib real

%% Graph Construct
m = [mb; m];
I = [Ib; I];
Lc = [0; Lc];    % may use Lcb instead of 0
ComPos = [ Lc, zeros(nb, 2)];   
L = [0; 0; L(1:end-1)];         % first row: zeros required
JointPos = [ L, zeros(nb, 2)];  % first row: zeros required
% JointType = {'R', 'Rx', 'Ry', 'Ry', 'Ry', 'Rx', 'Ry', 'Ry', 'Ry'}; %later
Ic = [ zeros(nb, 2), I ];       % only allowing z-rot inertia


%% Model Construct
model.NB = nb;
model.parent = parent;

for i=1:nb
%     model.jtype{i} = JointType{i};          % joint type
    model.jtype{i} = 'Rz';
    model.com{i}   = ComPos(i,:);           % Com Position
    model.mass{i} = m(i);                   % Body Mass 
    model.I{i} = mcI(model.mass{i}, model.com{i}, diag(Ic(i,:))); % Inertia
    model.Xtree{i} = plux(eye(3), JointPos(i,:));   % Tree transform
end

%% floating-base Expansion
floatingModel = floatbase(model);


end




