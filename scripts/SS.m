%Sparse Stereo
clc;
clear;
close all;
data = readtable("ssData.dat");

trials = 30;

noise = unique(data.Noise)*100;


d = reshape(data.Distance(1:trials*3),[3 trials])';


for i = (trials*3)+1:trials*3:length(data.Distance)
    distance = data.Distance(i:i+(trials*3-1));
    d = [d; reshape(distance,[3 trials])'];
end
d1 = d(:,1);
d2 = d(:,2);
d3 = d(:,3);

dist1 = d1(1:trials);
dist2 = d2(1:trials);
dist3 = d3(1:trials);
for i = trials+1:trials:length(d1)
    dist1 = [dist1 d1(i:i+(trials-1))];
    dist2 = [dist2 d2(i:i+(trials-1))];
    dist3 = [dist3 d3(i:i+(trials-1))];
end
mean_d1 = mean(dist1)';
mean_d2 = mean(dist2)';
mean_d3 = mean(dist3)';
figure(1);
hold on;
plot(noise,mean_d1,'or', 'MarkerFaceColor', 'r', 'MarkerSize',8);
plot(noise,mean_d2,'og', 'MarkerFaceColor', 'g', 'MarkerSize',8);
plot(noise,mean_d3,'ob', 'MarkerFaceColor', 'b', 'MarkerSize',8);
ylim([-0.005 0.07]);
xlim([-1 35]);
title("Distance from ground truth at different noise levels");
xlabel("Noise [%]");
ylabel("Distance [m]");
legend("Red cylinder","Green cylinder","Blue cylinder");
set(0,'defaultfigurecolor',[1 1 1])


set(gcf, 'PaperPosition', [-3.5 0 40 20])
set(gcf, 'PaperSize', [33 20]);
saveas(gcf, 'ss_noise', 'pdf')
