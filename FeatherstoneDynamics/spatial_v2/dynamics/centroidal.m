function  centr = centroidal( model, q, qd)

% EnerMo  calculate energy, momentum and related quantities
% EnerMo(robot,q,qd)  returns a structure containing the fields KE, PE,
% htot, Itot, mass, cm and vcm.  These fields contain the kinetic and
% potential energies of the whole system, the total spatial momentum, the
% total spatial inertia, total mass, position of centre of mass, and the
% linear velocity of centre of mass, respectively.  Vector quantities are
% expressed in base coordinates.  PE is defined to be zero when cm is
% zero.

for i = 1:model.NB
    [ XJ, S ] = jcalc( model.jtype{i}, q(i) );
    XJi{i} = XJ;
    Si{i} = S;
    vJ = S*qd(i);
    Xup{i} =  (XJ * model.Xtree{i});
    if model.parent(i) == 0
        v{i} = vJ;
    else
        v{i} =  (Xup{i}*v{model.parent(i)} + vJ);
    end
    Ic{i} = model.I{i};
    hc{i} =  (Ic{i} * v{i});
    %   KE(i) =  (0.5 * v{i}' * hc{i});
    %     disp(i)
end
centr.Xup{i} = Xup{i};
centr.Itot = zeros(size(Ic{1}));
centr.htot = zeros(size(hc{1}));

for i = model.NB:-1:1
    if model.parent(i) ~= 0
        Ic{model.parent(i)} =  (Ic{model.parent(i)} +  (Xup{i}'*Ic{i}*Xup{i}));
        hc{model.parent(i)} =  (hc{model.parent(i)} +  (Xup{i}'*hc{i}));
    else
        centr.Itot =  (centr.Itot +  (Xup{i}'*Ic{i}*Xup{i}));
        centr.htot =  (centr.htot +  (Xup{i}'*hc{i}));
    end
    %   disp(i)
end

a_grav = get_gravity(model);
if length(a_grav) == 6
    g = a_grav(4:6);			% 3D linear gravitational accn
    h = centr.htot(4:6);			% 3D linear momentum
else
    g = a_grav(2:3);			% 2D gravity
    h = centr.htot(2:3);			% 2D linear momentum
end

[mass, cm] = mcI(centr.Itot);

model.XbG = [ eye(3), zeros(3); -skew(cm), eye(3)];
model.XGb = [ eye(3), zeros(3); skew(cm), eye(3)];
hG = zeros(6,1);

for i = 1:model.NB
    if model.parent(i) ~= 0
        model.XiG{i} =  (Xup{i}*model.XiG{model.parent(i)});
        AG{i} =  (transpose(model.XiG{i})*Ic{i}*Si{i});
        hG =  (hG + AG{i}*qd(i));
    else
        model.XiG{i} = model.XbG;
        AG{i} =  (transpose(model.XiG{i})*Ic{i}*Si{i});
        hG =  (hG + AG{i}*qd(i));
    end
    %   disp(i)
end
centr.XiG = model.XiG;
centr.mass = mass;
centr.cm = cm;
centr.hG = hG;

% IG = (transpose(model.XbG)*centr.Itot*model.XbG);
XGb = inv(model.XbG);
IG = transpose(XGb)*centr.Itot*XGb;
IG3 = IG(1:3, 1:3);

centr.IG = IG;
% centr.vG = centr.IG\hG;


% ITorsoCM = mcI(model.mass{1}, cm, model.Ib);
% ITorsoCM = model.Ib;
if isfield(model, 'IGnominal')
    ITorsoCM = model.IGnominal;
    ITorsoCM3 = ITorsoCM(1:3, 1:3);
    ux = [1;0;0];
    uy = [0;1;0];
    uz = [0;0;1];
    
%     centr.CII = det(eye(3)-inv(IG3)*ITorsoCM3);
%     centr.CIIx = 1 - ux'*ITorsoCM3*ux/(ux'*IG3*ux);
%     centr.CIIy = 1 - uy'*ITorsoCM3*uy/(uy'*IG3*uy);
%     centr.CIIz = 1 - uz'*ITorsoCM3*uz/(uz'*IG3*uz);
    
%     centr.CII = det(eye(3)-inv(ITorsoCM3)*IG3);
%     centr.CII = abs(det(eye(3)-inv(IG3)*ITorsoCM3));
    centr.CII = (det(inv(ITorsoCM3)*IG3-eye(3))); 
%     centr.CII = (det(eye(3)-inv(IG3)*ITorsoCM3));

%     centr.CII = det(inv(IG3)*ITorsoCM3);
    centr.CIIx = 1 - ux'*IG3*ux/(ux'*ITorsoCM3*ux);
    centr.CIIy = 1 - uy'*IG3*uy/(uy'*ITorsoCM3*uy);
    centr.CIIz = 1 - uz'*IG3*uz/(uz'*ITorsoCM3*uz);
   
end
