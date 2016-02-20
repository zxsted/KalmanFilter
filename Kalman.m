
clear;

%Gets data from file
fileID = fopen('Accel.txt');
C = textscan(fileID,'%f');
fclose(fileID);

model_meas = cell2mat(C)';
size = size(model_meas);
num_of_meas = size(2);


dt = 1.0; %Time Step

x = [0;
     0]; %State Matrix
P = [.5, 0; 
     0, .5]; %State Estimate Covariance Matrix
F = [1, dt;
     0, 1 ]; %State Transistion Matrix

u = []; %Control Inpts
B = []; %Control Input Transistion Matrix

Q = [.00000000001, .0000000025; .000000025, .00005]; %Process Covariance Matrix
R = [.38]; %Measurement Covariance Matrix

H = [ 1, 0]; %Measuremnt Transistion Matrix
z = []; %Measurement

kalman_pos = [];
pred_pos=[];

%Basic Test Model: 0 Proccess Variance, Init Pos = 0; Vel = 1 m/s
%model_meas = ones(1,100);
% for i = 1:100
%    model_meas(i) = i + ((rand()*2)-1)*sqrt(R);
% end

%Runs the Kalman Filter
for i = 1:num_of_meas
    
    %---Kalman Filter------------------------------------------------------
    %Predict
    x_pred = F*x; %+ B*u;
    P_pred = F*P*F' + Q;
    
    pred_pos(i) = x_pred(1); 
    
    %Update
    
    z = model_meas(i);
    y = z - H*x_pred;       %Residual
    
    S = H*P_pred*H' + R;    %Innovtion Covaraince
    
    K = (P_pred*H')/S;      %Kalman Gain
    
    x = x_pred + K*y;          %Updated State
    P = (eye(2)-K*H)*P_pred;   %Updated State Estimate Covariance
    
    kalman_pos(i) = x(1);
    %--_End Kalman Filter--------------------------------------------------
    
end;

t = 1:num_of_meas;
plot( t, kalman_pos, 'r', t, model_meas, 'k');
