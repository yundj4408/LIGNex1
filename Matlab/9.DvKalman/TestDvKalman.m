clear all


dt = 0.1;
t  = 0:dt:10;

Nsamples = length(t);

Xsaved = zeros(Nsamples, 2);
Zsaved = zeros(Nsamples, 2);

for k=1:Nsamples
  [z, rV] = GetPos();      
  [pos, vel] = DvKalman(z);
  
  Xsaved(k, :) = [pos vel];
  Zsaved(k, :)    = [z rV];
end


figure
hold on
plot(t, Zsaved(:, 1), 'r.')
plot(t, Xsaved(:, 1))

figure
hold on
plot(t, Zsaved(:, 2), 'b--')
plot(t, Xsaved(:, 2))