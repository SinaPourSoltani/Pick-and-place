%
clc;
clear;
close all;

data = readtable("TrajectoryP2P.dat");
t = data{:,1}
titles = ["Position x" "Position y" "Position z" "Velocity x" "Velocity y" "Velocity z" "Acceleration x" "Acceleration y" "Acceleration z"]
ylabels = ["Position" "Velocity" "Acceleration"]
yidxs = [1 1 1 2 2 2 3 3 3]

str = '\color{red}[m]      \color{green}[m/s]      \color{blue}[m/s^2]'
plt(1,t,[data{:,2} data{:,5} data{:,8}],str , {'x position' 'x velocity' 'x acceleration'}, 'Position, velocity and aceleration for x')
saveFigureAsPDF(gcf, 'trajectory_p2p_x');
plt(2,t,[data{:,3} data{:,6} data{:,9}], str, {'y position' 'y velocity' 'y acceleration'}, 'Position, velocity and aceleration for y')
saveFigureAsPDF(gcf, 'trajectory_p2p_y');
plt(3,t,[data{:,4} data{:,7} data{:,10}], str, {'z position' 'z velocity' 'z acceleration'}, 'Position, velocity and aceleration for z')
saveFigureAsPDF(gcf, 'trajectory_p2p_z');


function saveFigureAsPDF(fig, path)
set(fig, 'PaperPosition', [0 0 35 20])
set(fig, 'PaperSize', [35 20]);
saveas(fig, path, 'pdf')
end


function plt(figureNumber, xx, yy, ylab, legendText, tlt)
figure(figureNumber)
hold on;
h = zeros(1,3);
h(1) = plot(xx,yy(:,1), 'Color','r', 'DisplayName',string(legendText(1)))
h(2) = plot(xx,yy(:,2), 'Color','g', 'DisplayName',string(legendText(2)))
h(3) = plot(xx,yy(:,3), 'Color','b', 'DisplayName',string(legendText(3)))
title(tlt);
ylimits = ylim;
diff = abs(ylimits(2)-ylimits(1))/20;
text(4,ylimits(1)+diff,'Red cylinder');
text(11.5,ylimits(1)+diff,'Green cylinder');
text(20,ylimits(1)+diff,'Blue cylinder');
xlim([0 23.9]);
xlabel('Time [s]');
ylabel(ylab);
vlines = [[8; 8] [16; 16]];
line(vlines(:,1)', ylim, 'Color', '#7E2F8E', 'LineStyle', '--');
line(vlines(:,2)', ylim, 'Color', '#7E2F8E', 'LineStyle', '--');
legend(h(1:3));
end
