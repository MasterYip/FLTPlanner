function [fitresult, gof] = fit_bordersize_searchtime(bordersize, tottime)
%CREATEFIT(BORDERSIZE,TOTTIME)
%  创建一个拟合。
%
%  要进行 '拟合曲线' 拟合的数据:
%      X 输入: bordersize
%      Y 输出: tottime
%  输出:
%      fitresult: 表示拟合的拟合对象。
%      gof: 带有拟合优度信息的结构体。
%
%  另请参阅 FIT, CFIT, SFIT.

%  由 MATLAB 于 09-May-2024 13:19:49 自动生成


%% 拟合: '拟合曲线'。
[xData, yData] = prepareCurveData( bordersize, tottime );

% 设置 fittype 和选项。
ft = fittype( 'exp1' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [0.0519533690599368 0.0300635867225613];

% 对数据进行模型拟合。
[fitresult, gof] = fit( xData, yData, ft, opts );

% 绘制数据拟合图。
figure( 'Name', '搜索时间-交线边界长度关系图' );
h = plot( fitresult, xData, yData );
legend( h, '搜索时间-交线边界长度', '指数拟合曲线', 'Location', 'NorthEast', 'Interpreter', 'none' );
% 为坐标区加标签
xlabel( '边界交线长度(unit)', 'Interpreter', 'none' );
ylabel( '搜索时间(ms)', 'Interpreter', 'none' );
grid on


