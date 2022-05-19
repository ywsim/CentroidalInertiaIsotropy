function JacoContact = contactJaco(model, q, qdot, flagFloat, contactBodyNum, contactPos)
% JacoContact gives Contact Jacobian in the base frame of the model. 
% (model) can be either fixed or floating based
% (flagFloat) accounts for the 6 additional bodies, affects contact number.
% (contactBodyNum) the number of body experiencing contact. In case of
% floating-based model, 6 additional bodies are neglected
% (contactPos) 3D vector to the contact point in the coordinate of the body
% experiencing the contact

nb = model.NB;
k = contactBodyNum;
if flagFloat
    k = k+6;
end

if (length(q)~=nb)
    disp('Dim Mismatch')
end

[v, Xup, PI_Xup_Contact] = vCalc(model, q, qdot, k);      % get vel transitions at the end
Xe = plux(eye(3), contactPos);          % up trf from Joint K to the contact pt
Xbe = Xe*PI_Xup_Contact;                % up trf from base to contact pt
[E, pbe] = plux(Xbe);                   % get ee pos/ori w.r.t. base
% JacoContact = simplify(jacobian(pbe, q));                % get ee vel/omega w.r.t. base
JacoContact = jacobian(pbe, q);                % get ee vel/omega w.r.t. base

end