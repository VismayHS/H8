%% TASK 3b - Compare model classes and select the best regressor.
%
%  WHY THIS EXISTS
%  The first attempt used a fitnet([12 8]) trained with Levenberg-Marquardt.
%  On the test set it scored R^2 = 0.52 / 0.55 / 0.045 for Kp / Ki / Kd -
%  worse than a straight line on a single feature. With 105 training samples
%  and roughly 200 network parameters it simply overfitted; validation error
%  bottomed out at epoch 7.
%
%  Choosing a model class by comparing candidates on held-out data is the
%  correct procedure anyway, so this script trains four and selects on merit:
%     1. linear least squares
%     2. quadratic (linear + squared terms + mass-relevant interactions)
%     3. small network with Bayesian regularisation (trainbr)
%     4. the original fitnet, for reference
%
%  Run after task3_generate_data_train_ml.m

clear; clc; rng(0);
L = load('task3_dataset.mat');
X = L.X;  Y = L.Y;  featNames = L.featNames;  PID_base = L.PID_base;
n = size(X,1);
gn = {'Kp','Ki','Kd'};

fprintf('=== TASK 3b: model class comparison ===\n');
fprintf('%d samples, %d features, %d targets\n\n', n, size(X,2), size(Y,2));

% identical split for every candidate, so the comparison is fair
idx = randperm(n);
nTr = round(0.70*n); nVa = round(0.15*n);
iTr = idx(1:nTr); iVa = idx(nTr+1:nTr+nVa); iTe = idx(nTr+nVa+1:end);
fprintf('split: %d train / %d val / %d test\n\n', numel(iTr), numel(iVa), numel(iTe));

Xtr = X(iTr,:); Ytr = Y(iTr,:);
Xte = X(iTe,:); Yte = Y(iTe,:);

R2 = @(yh,y) 1 - sum((yh-y).^2)./sum((y-mean(y)).^2);

cand = struct('name',{},'pred',{},'r2',{});

%% 1 - linear least squares
A  = [ones(nTr,1) Xtr];
Wl = A\Ytr;
predLin = @(xx) max([ones(size(xx,1),1) xx]*Wl, 0);
cand(end+1) = struct('name','linear', 'pred',{predLin}, ...
                     'r2', R2(predLin(Xte), Yte));

%% 2 - quadratic expansion
    function Z = expand(xx)
        Z = [xx, xx.^2, ...
             xx(:,6).*xx(:,4), ...    % thrustRatio x SSE
             xx(:,6).*xx(:,1), ...    % thrustRatio x overshoot
             xx(:,4).*xx(:,8)];       % SSE x stepSize
    end
Aq = [ones(nTr,1) expand(Xtr)];
Wq = Aq\Ytr;
predQuad = @(xx) max([ones(size(xx,1),1) expand(xx)]*Wq, 0);
cand(end+1) = struct('name','quadratic', 'pred',{predQuad}, ...
                     'r2', R2(predQuad(Xte), Yte));

%% 3 - small net, Bayesian regularisation
%  trainbr penalises large weights, which is the standard remedy for a small
%  dataset. No validation set is used (regularisation replaces early
%  stopping), so train and validation indices are pooled.
netBR = fitnet(6, 'trainbr');
netBR.trainParam.showWindow = false;
netBR.trainParam.epochs = 400;
netBR.divideFcn = 'dividetrain';
iFit = [iTr iVa];
netBR = train(netBR, X(iFit,:)', Y(iFit,:)');
predBR = @(xx) max(netBR(xx')', 0);
cand(end+1) = struct('name','NN-6 trainbr', 'pred',{predBR}, ...
                     'r2', R2(predBR(Xte), Yte));

%% 4 - original architecture, for reference
netLM = fitnet([12 8], 'trainlm');
netLM.trainParam.showWindow = false;
netLM.divideFcn = 'divideind';
netLM.divideParam.trainInd = 1:numel(iTr);
netLM.divideParam.valInd   = numel(iTr)+(1:numel(iVa));
netLM.divideParam.testInd  = [];
netLM = train(netLM, X([iTr iVa],:)', Y([iTr iVa],:)');
predLM = @(xx) max(netLM(xx')', 0);
cand(end+1) = struct('name','NN-[12 8] trainlm', 'pred',{predLM}, ...
                     'r2', R2(predLM(Xte), Yte));

%% report
fprintf('%-22s %8s %8s %8s %9s\n','model','R2 Kp','R2 Ki','R2 Kd','mean R2');
fprintf('%s\n', repmat('-',1,60));
best = 1; bestScore = -inf;
for i = 1:numel(cand)
    m = mean(cand(i).r2);
    fprintf('%-22s %8.3f %8.3f %8.3f %9.3f\n', cand(i).name, cand(i).r2, m);
    if m > bestScore, bestScore = m; best = i; end
end
fprintf('%s\n', repmat('-',1,60));
fprintf('SELECTED: %s (mean R2 = %.3f)\n\n', cand(best).name, bestScore);

predict_gains = cand(best).pred;
modelType     = cand(best).name;

% RMSE / MAE for the winner
Yh = predict_gains(Xte);
fprintf('Winner, per-gain test error:\n');
fprintf('%-5s %9s %9s %8s\n','gain','RMSE','MAE','R2');
for j = 1:3
    e = Yh(:,j)-Yte(:,j);
    fprintf('%-5s %9.3f %9.3f %8.3f\n', gn{j}, sqrt(mean(e.^2)), ...
            mean(abs(e)), cand(best).r2(j));
end

save('task3_model.mat','predict_gains','modelType','featNames', ...
     'PID_base','X','Y','iTr','iVa','iTe','netBR','Wl','Wq');
fprintf('\nSaved -> task3_model.mat (model: %s)\n', modelType);
