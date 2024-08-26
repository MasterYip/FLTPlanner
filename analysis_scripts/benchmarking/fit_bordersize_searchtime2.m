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
plot(x, fitresult.a*exp(fitresult.b*x), "LineWidth",2, "Color", [1, 0.7, 0.1])
% plot( fitresult ,'predobs', 0.95);
if en
    legend(  'Time - Border Length', 'Exponential fit', 'Lower bound', 'Upper bound', 'Location', 'NorthWest', 'Interpreter', 'none' );
    % 为坐标区加标签
    xlabel( 'Border Length(unit)', 'Interpreter', 'none' );
    % ylabel( 'Time(ms)', 'Interpreter', 'none' );
else
    legend(  '搜索时间-交线边界长度', '指数拟合曲线', '下界(指数拟合曲线)', '上界(指数拟合曲线)', 'Location', 'NorthWest', 'Interpreter', 'none' );
    % 为坐标区加标签
    xlabel( '边界交线长度(unit)', 'Interpreter', 'none' );
    % ylabel( '搜索时间(ms)', 'Interpreter', 'none' );
end
hold off;


