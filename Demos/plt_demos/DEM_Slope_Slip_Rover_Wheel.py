# -*- coding: utf-8 -*-
"""
Created on Wed Jan 31 18:43:25 2024

@author: 17908
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from numpy import polynomial as P
import math
from matplotlib import rcParams

DPI=300
FaceColor = 'w'

config = {
    "font.family":'Times New Roman',  # 设置字体类型
    "axes.unicode_minus": False #解决负号无法显示的问题
}
rcParams.update(config)
matplotlib.rcParams.update({'font.weight': 'bold'})
plt.rcParams ["font.weight"] = "bold" 
plt.rcParams ["axes.labelweight"] = "bold"


slip = [0.0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8]
Rover_Test1 = [-8.86 , 1.60 , 6.81 , 11.65 , 16.23 , 21.29 , 29.31 , 42.40 , 62.82]

Wheel_Test1 = [-17.16 , -5.25 , 2.33 , 7.76 , 12.81 , 19.24 , 28.22 , 42.06 , 65.94]



plt.plot(slip,Rover_Test1[:] ,label='Rover under DEM',linewidth=1,color='red',marker='*',linestyle='-',
        markerfacecolor='red',markersize=10)
plt.plot(slip,Wheel_Test1[:],label='Wheel under DEM',linewidth=1,color='blue',marker='d',linestyle='-',
        markerfacecolor='blue',markersize=10)

plt.xlim((-0.1, 0.9))
plt.ylim((-30, 100))
plt.grid(linestyle = '--')
plt.xlabel('Slip')
plt.ylabel('DrawBar-Pull (N)')
#plt.title('Rover vs. Single Wheel',fontweight = 'bold')
plt.legend(frameon=True,loc="upper left",fontsize='small',ncol=4)

dir = "D:/HuaweiMoveData/Users/17908/Desktop/Data/py_deomos/06_plot_rover_wheel/"
name='DBP_Slip_DEM.png'
plt.savefig(dir + name, facecolor = FaceColor, dpi = DPI)

plt.show()





tot_load_rover = 9.8 * 20.0
tot_load_wheel = 9.8 * 20.0
slope_rover = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
for i in range(9):
    slope_rover[i]= 180 / math.pi * np.arctan(1.0 / tot_load_rover * Rover_Test1[i]) 
    
slope_wheel = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
for i in range(9):
    slope_wheel[i]= 180 / math.pi * np.arctan(1.0 / tot_load_wheel * Wheel_Test1[i]) 
    
    
plt.plot(slip,slope_rover[:],label='Rover under DEM',linewidth=1,color='red',marker='*',linestyle='-',
        markerfacecolor='red',markersize=10)
plt.plot(slip,slope_wheel[:],label='Wheel under DEM',linewidth=1,color='blue',marker='d',linestyle='-',
        markerfacecolor='blue',markersize=10)

plt.xlim((-0.1, 0.9))
plt.ylim((-10, 30))
plt.grid(linestyle = '--')
plt.xlabel('Slip')
plt.ylabel('Slope (deg)')
#plt.title('Rover vs. Single Wheel',fontweight = 'bold')
plt.legend(frameon=True,loc="upper left",fontsize='small',ncol=4)

dir = "D:/HuaweiMoveData/Users/17908/Desktop/Data/py_deomos/06_plot_rover_wheel/"
name='Slope_Slip_DEM.png'
plt.savefig(dir + name, facecolor = FaceColor, dpi = DPI)

plt.show()