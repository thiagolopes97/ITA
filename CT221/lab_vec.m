% Import of data
VALE3(any(ismissing(VALE3),2), :) = [];
VALE3(VALE3.Volume == 0, :) = [];
dataset = VALE3(:, {'Date', 'Close'});

vec_num = 10; % Output size
look_back = 10; % Input size

% Making training and testing points
P = zeros(look_back, height(dataset)-vec_num-look_back+1); 
T = zeros(vec_num, height(dataset)-vec_num-look_back+1);
for i=vec_num:1:height(dataset)-look_back
    for j=1:1:look_back
        P(j, i-look_back+1) = dataset.Close(i+look_back-vec_num+1-j);
        T(j, i-vec_num+1) = dataset.Close(i+vec_num+1-j);
    end
end
 
%%%%Building Neural Network MLP
% Start of NN
net = feedforwardnet([25,25]);
net = configure(net, P, T);

% Pre and Pos processing
net.inputs{1}.processParams{2}.ymin = 0;
net.inputs{1}.processParams{2}.ymax = 1;
net.outputs{2}.processParams{2}.ymin = 0;
net.outputs{2}.processParams{2}.ymax = 1;
 
% Last 90 days to test
test_size = (90+10)/990;
net.divideFcn = 'divideblock';
net.divideParam.trainRatio = 1-test_size;
net.divideParam.testRatio = test_size;
 
% Configuration hidden layer
net = init(net);
net.layers{1}.dimensions = 30;
net.layers{1}.transferFcn = 'poslin'; %tansig, poslin
net.layers{2}.dimensions = 30;
net.layers{2}.transferFcn = 'poslin'; %tansig, poslin
 
% Training configuration
net.performFcn = 'mse';
net.trainFcn = 'trainlm'; % trainrp, trainlm, traincgp, trainbr 
net.trainParam.epochs = 1000000; 
net.trainParam.lr = 0.2;
net.trainParam.time = 1200;


% Training  
[net, tr] = train(net, P, T);
saida = net(P);

qntTest = size(tr.testInd);
qntTest = qntTest(2);
previsto = zeros(1, qntTest);
for t = tr.testInd
    idx = t-tr.testInd(1) + 1;
    for j=1:1:10
        previsto(idx) = previsto(idx) + saida(11-j,t-j);
    end
end
previsto = previsto ./10;
% Visualizing the prediction
plot(tr.testInd, dataset.Close(tr.testInd), 'r');
hold on
plot(tr.testInd,previsto,"LineStyle","--","Color","#999999");
legend('True', 'Prediction',  'Location' , 'northeast' );
ylabel('Close value prediction VALE3')
xlabel('Time point index from initial data (18/08/19)');

perf = perform(net,dataset.Close(tr.testInd), previsto)
