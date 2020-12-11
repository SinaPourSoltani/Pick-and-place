% RRT boxplots
clc;
clear;
close all;

data = readtable("RRTData.dat")

trials = 30;

nodes = data.nodes;
distance = data.distance;
extend = data.extend;
time = data.time;

t = [time(1:trials)];
n = [nodes(1:trials)];
d = [distance(1:trials)];

for i = trials+1:trials:length(time)
    t = [t time(i:i+(trials-1))];
    n = [n nodes(i:i+(trials-1))];
    d = [d distance(i:i+(trials-1))];
end

x = unique(extend);

bxplt(1,t,x,'Extend [rad]','Time [s]', 'Time vs extend')
saveFigureAsPDF(gcf, 'rrt_time_extend');
bxplt(2,n,x,'Extend [rad]','Nodes', 'Nodes vs extend')
saveFigureAsPDF(gcf, 'rrt_nodes_extend');
bxplt(3,d,x,'Extend [rad]','Distance [rad]', 'Distance vs extend')
saveFigureAsPDF(gcf, 'rrt_distance_extend');

function saveFigureAsPDF(fig, path)
set(fig, 'PaperPosition', [-3.5 0 40 20])
set(fig, 'PaperSize', [33 20]);
saveas(fig, path, 'pdf')
end

function bxplt(figureNumber, yx, xx, xlb, ylb, tlt)
    figure(figureNumber);
    clear figure;
    boxplot(yx);
    get(gca, 'XTickLabel');
    set(gca, 'XTickLabel', xx);
    xtickangle(90);
    xlabel(xlb);
    ylabel(ylb);
    title(tlt);
end
