
clear;
dt = 1.0; %Time Step

x = [50;
     0]; %State Matrix
P = [500, 0; 
     0, 500]; %State Estimate Covariance Matrix
F = [1, dt;
     0, 1 ]; %State Transistion Matrix

u = []; %Control Inpts
B = []; %Control Input Transistion Matrix

Q = [0]; %Process Covariance Matrix
R = [5]; %Measurement Covariance Matrix

H = [ 1, 0]; %Measuremnt Transistion Matrix
z = []; %Measurement

%Basic Test Model: 0 Proccess Variance, Init Pos = 0; Vel = 1 m/s
model_meas = ones(1,100);
for i = 1:100
    model_meas(i) = i + ((rand()*2)-1)*sqrt(R);
end

kalman_pos = [];
pred_pos=[];

%Runs the Kalman Filter
for i = 1:100
    
    %---Kalman Filter------------------------------------------------------
    %Predict
    x_pred = F*x; %+ B*u;
    P_pred = F*P*F'; %+ Q;
    
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

t = 1:100;
plot(t, t, t, kalman_pos, 'r', t, pred_pos, 'y', t, model_meas, 'k');
