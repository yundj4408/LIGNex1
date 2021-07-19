%
% 
%
clear all

% time step
dt = 1;

% process and measrement noise
Qc = 150;                           
Q = Qc * [dt^3/3 dt^2/2; dt^2/2 dt];        %시스템 노이즈의 공분산 행렬
R = 30;

% state initialization
x0 = [2000; 10];                            %레이더에서 표적까지의 초기 거리 2000m, 거리 변화율 10m/s
P0 = 30 * eye(2);                           %초기 오차 공분산 

% KF initialization 
xbar = x0 + sqrt(P0) * randn(2,1);          %x^-k = Ax^-(k-1)
Pbar = P0;
Qf = Q;

% collections
X = [];
XHAT = [];
PHAT = [];
KK = [];
Z = [];
ZHAT = [];
SBAR = [];
TIME = [];

% 상태 공간 모델을 위한 시스템 행렬
F = [1 dt; 0 1];                % xk+1 = Axk + wk A=F
H = [1 0];                      % zk = Hxk + vk   H=H

for time = 0:100
    
    % 측정 모델
    z = H * x0 +  sqrt(R) * randn();                %측정값 
    
    % 측정 업데이트
    zhat = H * xbar;                                %
    S = H * Pbar * H' + R;                          %Pbar -> 오차공분산 HPk-H^T + R
    Phat = Pbar - Pbar * H' * inv(S) * H * Pbar;    %IV. 오차 공분산 계산
    K = Pbar * H' * inv(S);                         %II. 칼만 이득 계속
    xhat = xbar + K * (z - zhat);                   %III. 추정값 계산
    
    % 시간 업데이트
    xbar = F * xhat;                                %feedback으로 I. 추정값 예측
    Pbar = F * Phat * F' + Qf;                      %I. 오차 공분산 예측
    
    % 시스템 모델 운동 모델
    x = F * x0 + sqrt(Q) * randn(2,1);              %I. 추정값 예측
       
    % 결과값을 저장하고 Plotting하기 위해 값 저장.
    X = [X; x0'];
    XHAT = [XHAT; xhat'];
    PHAT = [PHAT; diag(Phat)'];
    Z = [Z; z'];
    ZHAT = [ZHAT; zhat'];
    SBAR = [SBAR; diag(S)'];
    TIME = [TIME; time];
    KK = [KK; K'];
    
    % for next step
    x0 = x;
    
end

% plotting 거리 및 거리 변화율의 실제 궤적 및 추정
figure, plot(TIME, X(:,1),'r', TIME, XHAT(:,1),'b'),title('x1(거리 r)')
figure, plot(TIME, X(:,2),'r', TIME, XHAT(:,2),'b'),title('x2(거리 변화율 r미분')
% 추정 오차와 표준편차 -> 추정 오차가 표준편차안에 있는것을 보니 칼만필터 역할 잘 하고 있다,
figure, plot(TIME, X(:,1)-XHAT(:,1),'r',TIME,sqrt(PHAT(:,1)),'b',TIME,-sqrt(PHAT(:,1)),'b'),title('x1(거리 r)')
figure, plot(TIME, X(:,2)-XHAT(:,2),'r',TIME,sqrt(PHAT(:,2)),'b',TIME,-sqrt(PHAT(:,2)),'b'),title('x2(거리 변화율 r미분')

%칼만 게인도 짧은 시간이 경과한 후에 정정상태로 돌임함을 알 수 있다.
figure, plot(TIME, Z(:,1)-ZHAT(:,1),'r',TIME,sqrt(SBAR(:,1)),'b',TIME,-sqrt(SBAR(:,1)),'b'),title('r')
figure, plot(TIME, KK(:,1),'r',TIME,KK(:,2),'b'),title('Gain')