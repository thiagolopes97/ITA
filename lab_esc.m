% Import of data
VALE3(any(ismissing(VALE3),2), :) = [];
VALE3(VALE3.Volume == 0, :) = [];
dataset = VALE3(:, {'Date', 'Close'});

vec_num = 1; % Output size
look_back = 10; % Input size

% Making training and testing points
P = zeros(look_back, height(dataset)-look_back); 
T = zeros(vec_num, height(dataset)-look_back);
for i=1:1:height(dataset)-look_back
    for j=1:1:look_back
        P(j, i) = dataset.Close(i+look_back-j);
    end
    T(i) = dataset.Close(i+look_back);
end
 
%%%%Building Neural Network MLP
% Start of NN
net = feedforwardnet([10,10]);
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
net.layers{1}.dimensions = 10;
net.layers{1}.transferFcn = 'poslin'; %tansig
net.layers{2}.dimensions = 7;
net.layers{2}.transferFcn = 'poslin'; %poslin
 
% Training configuration
net.performFcn = 'mse';
net.trainFcn = 'traincgp'; % trainrp, trainlm, traincgp, trainbr 
net.trainParam.epochs = 1000000; 
net.trainParam.lr = 0.2;
net.trainParam.time = 1200;


% Training
[net, tr] = train(net, P, T);
saida = net(P);

% Visualizing the prediction
plot(tr.testInd, T(tr.testInd), 'r');
hold on
plot(tr.testInd, saida(tr.testInd),"LineStyle","--","Color","#999999");
legend('True', 'Prediction',  'Location' , 'northeast' );
ylabel('Close value prediction VALE3')
xlabel('Time point index from initial data (18/08/19)');

perf = perform(net,T(tr.testInd), saida(tr.testInd))
