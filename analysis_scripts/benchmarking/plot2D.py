import matplotlib.pyplot as plt
from matplotlib import rcParams
from cycler import cycler
import pandas as pd
import scipy
import scipy.signal
import numpy as np
from scipy.interpolate import make_interp_spline
# import csv
# import glob
# import os

defaultConfig = {
    "mathtext.fontset": 'stix',
    'font.family': 'Times New Roman',
    # "font.serif":['cmex10'],
    "font.size": 15,
    'axes.unicode_minus': False,
    'axes.prop_cycle': (cycler('color', ['k', 'r', 'g', 'b'])
                        + cycler('linestyle', ['--', '-', '--', '-'])),
}


class CVSPlot:

    def __init__(self, cfg=defaultConfig):
        rcParams.update(cfg)

    def plot(self, CSV_FILE, index_x, index_y, xlabel, ylabel, title, label, axes=None):
        df = pd.read_csv(CSV_FILE)
        Y = df.iloc[1:, index_y]
        if index_x is not None:
            X = df.iloc[1:, index_x]
        else:
            X = np.arange(len(Y))
        if (axes):
            axes.plot(X, Y, label=label)
            axes.set_xlabel(xlabel)
            axes.set_ylabel(ylabel)
            axes.set_title(title)
        else:
            plt.plot(X, Y, label=label)
            plt.xlabel(xlabel)
            plt.ylabel(ylabel)
            plt.title(title)

    def scatter(self, CSV_FILE, index_x, index_y, marker, xlabel, ylabel, title, label):
        df = pd.read_csv(CSV_FILE)
        X = df.iloc[1:, index_x]
        Y = df.iloc[1:, index_y]
        plt.scatter(X, Y, marker=marker, label=label)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.title(title)

    def plot_with_filter(self, CSV_FILE, index_x, index_y, xlabel, ylabel, title, label):
        '''
        @brief: 使用Savitzky-Golay平滑滤波--对一定长度窗口内的数据点进行k阶多项式拟合, 从而得到拟合后的结果
        @param:
            - window: 窗口数量
            - poly: k阶多项式拟合
        '''
        df = pd.read_csv(CSV_FILE)
        Y = df.iloc[1:, index_y]
        if index_x is not None:
            X = df.iloc[1:, index_x]
        else:
            X = np.arange(len(Y))

        window = 50
        poly = 3
        tmp_smooth = scipy.signal.savgol_filter(Y, window, poly)

        plt.plot(X, Y, label=label)
        plt.plot(X, tmp_smooth, label=str("smooth " + label))
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.title(title)

    def sliding_average(self, CSV_FILE, index_x, index_y, xlabel, ylabel, title, label):
        '''
        @brief: 滑动窗口均值滤波--一维卷积
        @param:
            - N: 卷积核长度
        '''
        df = pd.read_csv(CSV_FILE)
        Y = df.iloc[1:, index_y]
        if index_x is not None:
            X = df.iloc[1:, index_x]
        else:
            X = np.arange(len(Y))

        N = 10
        kernel = np.ones(N) / N
        Y_smooth = np.convolve(Y, kernel, mode='same')
        plt.plot(X, Y, label=label)
        plt.plot(X, Y_smooth, label=str("smooth" + label))
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.title(title)

    def inter_spline(self, CSV_FILE, index_x, index_y, xlabel, ylabel, title, label):
        '''
        @brief: 插值平滑--适用于少量样本对折线进行平滑处理
        @param:
            - inter_num: 插值个数
        '''

        df = pd.read_csv(CSV_FILE)
        Y = df.iloc[1:, index_y]
        if index_x is not None:
            X = df.iloc[1:, index_x]
        else:
            X = np.arange(len(Y))

        inter_num = 400
        X_smooth = np.linspace(np.array(X).min(), np.array(X).max(), inter_num)
        Y_smooth = make_interp_spline(X, Y)(X_smooth)
        plt.plot(X, Y, label=label)
        plt.plot(X_smooth, Y_smooth, label=str("smooth" + label))
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.title(title)

    def get_mean(self, CSV_FILE, index_x, index_y, data_front, data_back, label):
        '''
        @brief: 求取mean
        @param:
            - inter_num: 插值个数
        '''
        df = pd.read_csv(CSV_FILE)
        Y = df.iloc[1:, index_y]
        if index_x is not None:
            X = df.iloc[1:, index_x]
        else:
            X = np.arange(len(Y))

        mean = np.mean(np.array(Y)[data_front:data_back])
        mean_Y = np.ones(len(X)) * mean
        # print(mean_Y.shape)
        plt.plot(X, Y, label=label)
        plt.plot(X, mean_Y, label="mean")
        # plt.title(title)
        return mean

    def update_cfg(self, cfg):
        rcParams.update(cfg)

if __name__ == "__main__":
    # csv_file_f = "/home/lenovo/MarsSim_v2_ws/src/rover_control/data_identify/identify_02/0.5_2.46_0.62/slip_FL.csv"
    # csv_file_m = "/home/lenovo/MarsSim_v2_ws/src/rover_control/data_identify/identify_02/0.5_2.46_0.62/slip_ML.csv"
    # csv_file_b = "/home/lenovo/MarsSim_v2_ws/src/rover_control/data_identify/identify_02/0.5_2.46_0.62/slip_BL.csv"
    # index_x = 0
    # index_y = 0
    # xlabel = "滑转率 s"
    # ylabel = "PC 牵引力系数"
    # title = "s-PC 曲线2"
    # plt.figure(figsize=(9, 6))

    # plot(CSV_FILE=csv_file, index_x=index_x, index_y=index_y, xlabel=xlabel, ylabel=ylabel,title=title)
    # sliding_average(CSV_FILE=csv_file, index_x=None, index_y=index_y,
    #          xlabel=xlabel, ylabel=ylabel,title=title, label="slip")
    # mean_f = get_mean(CSV_FILE=csv_file_f, index_x=None, index_y=index_y, data_front=500, data_back=2000, label="slip_m")
    # mean_m = get_mean(CSV_FILE=csv_file_m, index_x=None, index_y=index_y, data_front=500, data_back=2000, label="slip_m")
    # mean_b = get_mean(CSV_FILE=csv_file_b, index_x=None, index_y=index_y, data_front=500, data_back=2000, label="slip_m")
    # print('\033[;33m', round(mean_f, 2), round(mean_m, 2), round(mean_b, 2), '\033[;0m')

    # plt.legend()
    # plt.savefig(f"{title}.png", dpi=300, format="png", bbox_inches = 'tight')
    # plt.show()
    # file = "/home/lenovo/dataProcess/data/expriment/data1.csv"
    # plot(file, None, 0, "time", "slip", "vf=0.4", "slip")
    # mean1 = get_mean(file, None, 17, 150, 300, "sop")
    # mean1= get_mean(file, None, 18, 150, 300, "sop")
    # mean1 = get_mean(file, None, 19, 150, 300, "sop")
    # print(mean1)

    # file = r"D:\Graduation_project\dataProcess\data\expriment\17_0.614_1.463.csv"

    # plot_with_filter(file, None, 17, "time", "slip",
    #                  "slip vs time", "front wheel")
    # plot_with_filter(file, None, 18, "time", "slip",
    #                  "slip vs time", "middle wheel")
    # # plot_with_filter(file, None, 19, "time", "slip", "slip vs time", "rear wheel")
    # plt.legend(loc="lower right")
    # # plt.savefig(r"C:\Users\LENOVO_TL\Desktop\filter.png")
    # plt.show()
    pass
