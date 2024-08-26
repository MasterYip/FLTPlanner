function [fitresult, gof] = fit_bordersize_searchtime2(bordersize, tottime, en)
%CREATEFIT(BORDERSIZE,TOTTIME)
%  创建一个拟合。
%
%  要进行 '指数拟合曲线' 拟合的数据:
%      X 输入: bordersize
%      Y 输出: tottime
%  输出:
%      fitresult: 表示拟合的拟合对象。
%      gof: 带有拟合优度信息的结构体。
%
%  另请参阅 FIT, CFIT, SFIT.

%  由 MATLAB 于 09-May-2024 14:04:10 自动生成

% Curve fit
[xData, yData] = prepareCurveData( bordersize, tottime );
ft = fittype( 'a*exp(b*x)',...
            'independent','x');
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [0.08 0.03];
[fitresult, gof] = fit( xData, yData, ft, opts );

%% Plot
hold on;
x = 0:1:150;
% [intervals] = confint(fitresult)
% plot(x, 0.08*exp(intervals(1)*x), "LineWidth",2)
% plot(x, 0.08*exp(intervals(2)*x), "LineWidth",2)
plot(x, fitresult.a*exp(fitresult.b*x), "LineWidth",3, "Color", [1, 0.6, 0.0])
% plot( fitresult ,'predobs', 0.95);

hold off;


