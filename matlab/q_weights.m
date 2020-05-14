clear; clc

% Distance vector
l = [135.7, 425, 392.43, 93, 82, 0];
range= 9
last = 1


%% Percentage of rest

for i = 1:length(l)
   w(i) = (sum(l(i:end))/sum(l))*range + last
end
w

%% Diameter

for i = 1:length(l)
   w(i) = 2*pi*sum(l(i:end));
end

wsum = sum(w);
for i = 1:length(l)
   wi_correct = w(i)/wsum;
   w(i)=1+wi_correct;
end


