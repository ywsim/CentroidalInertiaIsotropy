classdef RBDyn3 < handle
    %RBDyn Generates the rigid body dynamics from a urdf file
    % ============================================== %
    properties
        robotStruc
        RbmFixed
        RbmFloat
        name
    end
    
    methods (Access = public)
        function obj = RBDyn3(strFileName, strName)
            obj.name = strName;
            obj.robotStruc = importrobot(strFileName);
            obj.robotStruc.DataFormat = 'column';
            
            nBodies = obj.robotStruc.NumBodies;
            obj.RbmFixed.NB = nBodies;
            obj.RbmFixed.mass = cell(1, nBodies);
            obj.RbmFixed.I = cell(1, nBodies);
            obj.RbmFixed.Xtree = cell(1, nBodies);
            obj.RbmFixed.jtype = cell(1, nBodies);
            
            for idxBody = 1:nBodies
                body_ = obj.robotStruc.Bodies{idxBody};
                
                mass = body_.Mass;
                com = body_.CenterOfMass;
                iCvec = body_.Inertia;
                iC = [iCvec(1), iCvec(6), iCvec(5); ...
                    iCvec(6), iCvec(2), iCvec(4); ...
                    iCvec(5), iCvec(4), iCvec(3)];
                obj.RbmFixed.mass{idxBody} = mass;
                obj.RbmFixed.I{idxBody} = mcI(mass, com, iC);
                
                disTree = body_.Joint.JointToParentTransform(1:3, 4);
                obj.RbmFixed.Xtree{idxBody} = plux(eye(3), disTree);
                
                jvec_ = body_.Joint.JointAxis;
                if prod(jvec_ == [1 0 0]) || prod(jvec_ == [-1 0 0])
                    obj.RbmFixed.jtype{idxBody} = 'Rx';
                elseif prod(jvec_ == [0 1 0]) || prod(jvec_ == [0 -1 0])
                    obj.RbmFixed.jtype{idxBody} = 'Ry';
                elseif prod(jvec_ == [0 0 1]) || prod(jvec_ == [0 0 -1])
                    obj.RbmFixed.jtype{idxBody} = 'Rz';
                end
            end
            
            for idxBody = 1:nBodies
%                 parentIdx = find(contains(obj.robotStruc.BodyNames, ...
%                     obj.robotStruc.Bodies{idxBody}.Parent.Name))
                parentIdxBool = strcmp(obj.robotStruc.BodyNames, ...
                                       obj.robotStruc.Bodies{idxBody}.Parent.Name);
                parentIdx = find(parentIdxBool == 1);
                if isempty(parentIdx)
                    obj.RbmFixed.parent(idxBody) = 0;
                else
                    obj.RbmFixed.parent(idxBody) = parentIdx;
                end
            end
            
            obj.RbmFloat = floatbase(obj.RbmFixed);
        end
        
        function out = getCentrInertia(obj, qFull)
            % initialize
            model = obj.RbmFloat;
            zeroCell = cell(1, model.NB);
            XJi = zeroCell;
            Xup = zeroCell;
            Ic = zeroCell;
            % get X:parent(i) -> i, and inertia
            for i = 1:model.NB
                model.jtype{i};
                [ XJ, ~ ] = jcalc( model.jtype{i}, qFull(i) );
                XJi{i} = XJ;
                Xup{i} =  (XJ * model.Xtree{i});
                Ic{i} = model.I{i};
            end
            
            % propagate inertia towards the base
            Itot = zeros(6, 6);
            for i = model.NB:-1:1
                if model.parent(i) ~= 0
                    Ic{model.parent(i)} =  (Ic{model.parent(i)} +  (Xup{i}'*Ic{i}*Xup{i}));
                else
                    Itot =  (Itot + (Xup{i}'*Ic{i}*Xup{i}));
                end
            end
            
            % get CoM vector in base frame
            [~, cm] = mcI(Itot);
            
            % translate Itot into CoM frame
            % model.XbG = [ eye(3), zeros(3); -skew(cm), eye(3)];
            XGb = [ eye(3), zeros(3); skew(cm), eye(3)];
            ItotG = transpose(XGb)*Itot*XGb;  % 6 x 6
%             model.centr.XGb = XGb;
            out = ItotG;
        end
        
        function out = genFullConfig(obj, qPartial)
            if size(qPartial, 1) ~= 3
                qPartial = qPartial';
            end
            len = size(qPartial, 2);
            q2 = qPartial(1,:);
            q3 = qPartial(2,:);
            q4 = qPartial(3,:);

            switch obj.name
                case 'MiniCheetah'
                    out = [zeros(6, len); ...
                           q2;q3;q4; ...
                          -q2;q3;q4; ...
                           q2;-q3;-q4; ...
                          -q2;-q3;-q4];
                case 'TelloCA'
                    out = [zeros(6, len); ...
                           zeros(1, len);   q2; q3; q4; -q3-q4; ...
                           zeros(1, len);  -q2; q3; q4; -q3-q4];
                case 'Cassie'
                    out = [zeros(6, len); ...
                           q2; zeros(3, len); q3; q4; -(q3 + q4) - pi/4; ...
                          -q2; zeros(3, len); q3; q4; -(q3 + q4) - pi/4];
                case 'Atlas'
                    out = [zeros(6, len); zeros(4, len); ...
                          -pi/2*ones(1, len);   zeros(6, len); ...
                           pi/2*ones(1, len);   zeros(5, len); ...
                           0*ones(1, len); q2;  q3;  q4; (-q3-q4).*ones(1, len); 0*ones(1, len); ... 
                           0*ones(1, len); -q2;  q3;  q4; (-q3-q4).*ones(1, len); 0*ones(1, len)];

                case 'TelloColA'
                    out = [zeros(6, len); ...
                           zeros(1, len);   q2; q3; q4; -q3-q4; ...
                           zeros(1, len);  -q2; q3; q4; -q3-q4];
                case 'HuboPlus'
                    out = [zeros(6,len); zeros(3,len); ...
                           0*ones(1, len);  q2; q3; q4; (-q3-q4).*ones(1, len); 0*ones(1, len);  ...
                           0*ones(1, len); -q2; q3; q4; (-q3-q4).*ones(1, len); 0*ones(1, len); ...
                           zeros(42, len)];
            end
        end
        
        function showRobot(obj, q)
            show(obj.robotStruc, q(6:end))
        end
        
        function out = calcCII(obj, qFull0, qFullTest)
            
            % Get CCRBI at nominal configuration, q0. (6x6)
            Ig_q0 = obj.getCentrInertia(qFull0);
            
           
            % Get RCCRBI, the rotational part of CCRBI (3x3)
            Ig_q0_rot = Ig_q0(1:3,1:3);
            
            
            % calculate CII for configurations q with respect to q0
            % a singular q is 3x1 vector, for N configurations, q is 3xN 
            
            len = size(qFullTest, 2);          % number of configurations 
            CII_ = zeros(1, len);
            
            for idxConfig = 1:len   % for all configurations
                
                % Calculate RCCRBI per configuration
                Ig_q = obj.getCentrInertia(qFullTest(:, idxConfig));
                Ig_q_rot = Ig_q(1:3, 1:3);
                
                % Calculate CII per configuration
                CII_(idxConfig) = det(Ig_q_rot\Ig_q0_rot-eye(3));
                
            end
                                           
            out.config = qFullTest;
            out.CiiValue = CII_;
        end
    end
end
