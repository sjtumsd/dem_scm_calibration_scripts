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
Rover_DBP_SCM = [-65.77 , -1.38 , 94.69 , 156.66 , 210.63 , 249.93 , 297.39 , 352.35 , 401.48] 
Rover_DBP_SCM_NOSHADOW = [-75.57 , -68.38 , -1.94 , 52.81, 110.97 , 176.19 , 241.51 , 308.25 , 382.11]

Rover_Test_former = [49.7 , 132.72 , 186.81 , 224.07 , 256.76 , 289.72 , 326.28 , 365.98 , 406.83]
Rover_Test1_former = [-47.6 , -24.12 , 49.67 , 95.34 , 130.05 , 183.63 , 224.44 , 273.61 , 341.65]
Rover_Test1 = [-43.57 , -11.17 , 20.26 , 91.34 , 139.03 , 172.17 , 210.95 , 274.38 , 334.45]
Rover_Test2 = [-11.44 , 17.83 , 59.14 , 103.74 , 131.75 , 144.93 , 169.05 , 217.36 , 269.52]


Wheel_DBP_SCM = [-14.7 , -12 , -0.5 , 8.4 , 19.7 , 30.64 , 39.48 , 51.73 , 64.33]  

Wheel_Test_former = [10.27 , 20.18 , 27.61 , 33.21 , 38.12 , 44.60 , 51.29 , 59.11 , 66.71]
Wheel_Test1 = [-8.1 , -7.8 , 0.15 , 7.91 , 14.35 , 22.47 , 31.95 , 45.2 , 59.71]
Wheel_Test2 = [-1.8 , 0.27 , 5.62 , 12.75 , 16.53 , 23.21 , 26.813 , 35.82 , 45,95]




Rover_SCM = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
for i in range(9):
    Rover_SCM[i]= Rover_Test1[i] / 6.0
plt.plot(slip,Rover_SCM[:] ,label='Rover under SCM',linewidth=1,color='red',marker='*',linestyle='-',
        markerfacecolor='red',markersize=10)
plt.plot(slip,Wheel_Test1[:],label='Wheel under SCM',linewidth=1,color='blue',marker='d',linestyle='-',
        markerfacecolor='blue',markersize=10)

plt.xlim((-0.1, 0.9))
plt.ylim((-30, 100))
plt.grid(linestyle = '--')
plt.xlabel('Slip')
plt.ylabel('DrawBar-Pull (N)')
#plt.title('Rover vs. Single Wheel',fontweight = 'bold')
plt.legend(frameon=True,loc="upper left",fontsize='small',ncol=4)

dir = "D:/HuaweiMoveData/Users/17908/Desktop/Data/py_deomos/06_plot_rover_wheel/"
name='DBP_Slip_SCM.png'
plt.savefig(dir + name, facecolor = FaceColor, dpi = DPI)

plt.show()





tot_load_rover = 9.8 * 120.0
tot_load_wheel = 9.8 * 20.0
slope_rover = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
for i in range(9):
    slope_rover[i]= 180 / math.pi * np.arctan(1.0 / tot_load_rover * Rover_Test1[i]) 
    
slope_wheel = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
for i in range(9):
    slope_wheel[i]= 180 / math.pi * np.arctan(1.0 / tot_load_wheel * Wheel_Test1[i]) 
    
    
plt.plot(slip,slope_rover[:],label='Rover under SCM',linewidth=1,color='red',marker='*',linestyle='-',
        markerfacecolor='red',markersize=10)
plt.plot(slip,slope_wheel[:],label='Wheel under SCM',linewidth=1,color='blue',marker='d',linestyle='-',
        markerfacecolor='blue',markersize=10)

plt.xlim((-0.1, 0.9))
plt.ylim((-10, 30))
plt.grid(linestyle = '--')
plt.xlabel('Slip')
plt.ylabel('Slope (deg)')
#plt.title('Rover vs. Single Wheel',fontweight = 'bold')
plt.legend(frameon=True,loc="upper left",fontsize='small',ncol=4)

dir = "D:/HuaweiMoveData/Users/17908/Desktop/Data/py_deomos/06_plot_rover_wheel/"
name='Slope_Slip_SCM.png'
plt.savefig(dir + name, facecolor = FaceColor, dpi = DPI)

plt.show()