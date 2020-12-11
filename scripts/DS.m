%Depth Sensor
clc;
clear;
close all;
data = readtable("DepthSensorTestBIG.txt");

trials = 20;

noise = unique(data.Var1);
noise = noise(1:end-1)

noiseR = data.Var3;
noiseG = data.Var4;
noiseB = data.Var5;

d1 = [noiseR(1:trials)];
d2 = [noiseG(1:trials)];
d3 = [noiseB(1:trials)];

for i = trials+1:trials:length(data.Var3)-1
    d1 = [d1 noiseR(i:i+(trials-1))];
    d2 = [d2 noiseG(i:i+(trials-1))];
    d3 = [d3 noiseB(i:i+(trials-1))];
end

threshold = 0.1;

count1 = zeros(1,max(size(d1)));
count2 = zeros(1,max(size(d1)));
count3 = zeros(1,max(size(d1)));
for i = 1:max(size(d1))
   tmp1 = d1(:,i);
   count1(i) = size(tmp1(tmp1<=threshold),1);
   tmp2 = d2(:,i);
   count2(i) = size(tmp2(tmp2<=threshold),1);
   tmp3 = d3(:,i);
   count3(i) = size(tmp3(tmp3<=threshold),1);
end





count = [count1;count2;count3]';

count = count/60*100;

b = bar(count, 'stacked');
b(1).FaceColor = 'red';
b(2).FaceColor = 'green';
b(3).FaceColor = 'blue';
set(gca,'xlim',[0 max(size(count))+1])
set(gca, 'XTickLabel', noise*2);
xtickangle(90);
%ylim([0 trials*3+1])
legend('Red cylinder','Green cylinder','Blue cylinder')
xlabel('Noise [m]')
ylabel('Succesful centroid detection [%]')
title("Succesful centroid detection vs noise")

set(gcf, 'PaperPosition', [-3.5 0 40 20])
set(gcf, 'PaperSize', [33 20]);
saveas(gcf, 'ds_barplot', 'pdf')

unwanted = 1.3652;

for i = 1:max(size(d1))
    tmp = d1(:,i)
    tmp(tmp>=unwanted) = 0;
    d1(:,i) = tmp;
    tmp = d2(:,i)
    tmp(tmp>=unwanted) = 0;
    d2(:,i) = tmp;
    tmp = d3(:,i)
    tmp(tmp>=unwanted) = 0;
    d3(:,i) = tmp; 
end


mean_d1 = mean(d1);
mean_d2 = mean(d2);
mean_d3 = mean(d3);

figure(2);
hold on;
plot(noise,mean_d1,'or', 'MarkerFaceColor', 'r', 'MarkerSize',8);
plot(noise,mean_d2,'og', 'MarkerFaceColor', 'g', 'MarkerSize',8);
plot(noise,mean_d3,'ob', 'MarkerFaceColor', 'b', 'MarkerSize',8);
title("Noise vs error");
xlabel("Noise [m]");
ylabel("Error [m]");
legend("Red cylinder","Green cylinder","Blue cylinder");
set(0,'defaultfigurecolor',[1 1 1])

set(gcf, 'PaperPosition', [-3.5 0 40 20])
set(gcf, 'PaperSize', [33.2 20]);
saveas(gcf, 'ds_noise', 'pdf')
