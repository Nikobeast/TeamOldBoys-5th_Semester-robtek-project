%% Projekt script
%% This Part of the Script plots the marbles positions relative to the positions found by the Hough algorithm and by our own alogrithm
clc,clear;
ball_x=[10 -3 20 18.5 28 20 32 31 27 18 10 8.5 -9 39 41 25 18 6]; 
ball_y=[4 -1 23 7 12 23 23 7 -5 -7 -6 -6.5 -13 -15 -22 -23 -17 -19];
hough_x = [20 31 28 32 19 9 27 10 12 16 -8 38 19 7 40 25 ]; 
hough_y = [23 23 12 9 7 3 -5 -7 -7 -8 -13 -15 -17 -19 -21 -23 ];
our_x = [20 31 28 32 19 9 27 10 17 19 -9 39 19 6 40 25 ]; 
our_y = [23 23 12 8 7 4 -5 -7 -8 -8 -13 -15 -17 -19 -21 -23 ];
figure
hold on
legend
scatter(0,0,500,[0 0 0],'+');
scatter(ball_x,ball_y,36,[0 0 1],'filled');
scatter(hough_x,hough_y,50,[1 0 0],'*');
scatter(our_x,our_y,50,[0 1 0],'x');
hold off
legend('(0,0)','Marbels','Hough-detector','Our own detector','Location','NorthEastOutside')
%% Run this part of the script with the dataOut.txt file that is outputtet from the Q-Learn algortihm to plot the data.
clc,clear;
spacing=50; % This value MUST be the same as the spacing value stated in the Q-Learning Algorithm.
Data = csvread('dataOut.txt',0,1);
x=[1:1:length(Data(1,:))-3];
legendCell=strcat('\alpha=' , strtrim(cellstr(num2str(Data(:,1)))),' \gamma=',strtrim(cellstr(num2str(Data(:,2)))), ' \epsilon=',strtrim(cellstr(num2str(Data(:,3)))));
figure
hold on
for i = 1:length(Data(:,1))
        plot(spacing*x-spacing,Data(i,4:end), 'LineWidth', 1);
end
hold off
legend(legendCell)




