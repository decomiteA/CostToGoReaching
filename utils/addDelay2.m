function [Adel,Bdel,Hdel,x0del,Qdel,QNdel,OmegaXidel,OmegaOmdel,Sigdel] = addDelay2(A,B,H,x0,ndet1,ndet2,Q,QN,OmegaXi,OmegaOm,Sig)
%addDelay2 expands the matrices and vectors contained in the state space
%dynamical systems to integrate sensory delays associated with two different
% sensory modalities. Different observability matrices (H) can be defined
% for each sensory modality.
%
% INPUTS
% ======
% - A,B and H are the matrices of the dynamical system
% - x0 is the initial state
% - ndet1 and ndet2 are the amount of delay associated with each sensory
%   modality
% - Q and QN are the (final) and (running) error cost matrices
% - OmegaXi and OmegaOm are the motor and sensory noise covariance
%   matrices, respectively
% - Sig is the initial covariance matrix of the state estimate
%
%
% OUTPUTS
% =======
% All the outputs are the delayed counterpart of their associatd inputs, a
% "del" postfix was added to each of them
%
% @ Antoine de Comite - Sept 2023


maxdel = max([ndet1,ndet2]);

Adel = zeros(maxdel*size(A));
Bdel = zeros([maxdel*size(B,1) size(B,2)]);
Hdel = zeros([2*size(H,1),maxdel*size(H,2)]);
Qdel = zeros(maxdel*size(Q,1),maxdel*size(Q,2),size(Q,3));
QNdel = zeros(maxdel*size(QN));
OmegaXidel = zeros(maxdel*size(OmegaXi));
OmegaOmdel = eye(2*size(OmegaOm))*OmegaOm(1,1);
Sigdel = zeros(maxdel*size(Sig));

x0del = repmat(x0,maxdel,1);

Adel(1:size(A,1), 1:size(A,2)) = A;
Sigdel(1:size(Sig,1), 1:size(Sig,2)) = Sig;
for ii = 1 : size(Qdel,3)
    Qdel(1:size(QN,1),1:size(QN,2),ii) = Q(:,:,ii);
    Adel(size(A,1)+1:end,1:end-size(A,2)) = eye(size(Adel,2) - size(A,2));
    Bdel(1:size(B,1),:) = B;
    Hdel(1:size(H,1), (ndet1-1)*size(H,2)+1:ndet1*size(H,2)) = H; % Defining the observability matrix associated with the first sensory modality
    OmegaXidel(1:size(OmegaXi,1), 1:size(OmegaXi,2)) = OmegaXi;
    Hdel(size(H,1)+1:end, (ndet2-1)*size(H,2)+1:end) = H;         % Defining the observability matrix associated with the second sensory modality
    QNdel(1:size(QN,1), 1:size(QN,2)) = QN;
    OmegaOmdel(1:size(OmegaOm,1), 1:size(OmegaOm,2)) = OmegaOm;
end

end