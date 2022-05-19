function  [v, Xup, PIXupk] = vCalc( model, q, qd, k)

% vCalc  Calculates velocity and decendent transformations 
% Outputs:
% (v) are spatial velocities of rigid bodies in their body frames
% (Xup) are lam(i)-(i) 
for i = 1:model.NB
  [ XJ, S{i} ] = jcalc( model.jtype{i}, q(i) );
  vJ = S{i}*qd(i);
  Xup{i} = XJ * model.Xtree{i};
  if model.parent(i) == 0
    v{i} = vJ;
%   elseif i==k
%     v{i} = plux(eye(3), pk)*(Xup{i}*v{model.parent(i)} + vJ);  
  else
    v{i} = Xup{i}*v{model.parent(i)} + vJ;
    
  end
  
end
Xp = eye(6);
Ka = kappa(model.parent);
for jj = Ka{k}
    Xp = Xup{jj}*Xp;
end
PIXupk = simplify(Xp);

end
