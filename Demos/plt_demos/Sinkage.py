import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from numpy import polynomial as P
import math

# General information of the data and plots
lw = 2
fs = 14
ms = 14
DPI = 300
Ncol = 3
Weight = 'bold'
FigSize = [8, 6]
Radii = [0.2, 0.3]
plate_size = ["20cm", "30cm"]

LableName = ["Plate speed = 0.25 cm/s", "Plate speed = 0.5 cm/s", "Plate speed = 1.0 cm/s", "Selected points"]
LineStyle = ["-.", "--", ":", "-"]
Color = ["red", "cyan", "blue", "purple"]

# dir0 = "/srv/home/whu59"
# dir0 = "/home/whu59/research/server/euler"
# dir1 = "/research/sbel/d_chrono_fsi_granular/chrono_3001"
dir2 = "E:/GPU_FILE/Parameter_test/Plate_sinkage_test/"
dir3 = [["r=20cm/v025", "r=30cm/v025"],
        ["r=20cm/v05", "r=30cm/v05"],
        ["r=20cm/v10", "r=30cm/v10"]]
dir4 = "/DBP.txt"

out_dir = "01_plot_plate_sinkage/"

# Selected force at each sinkage
Force_Selected_Points_CRM = np.zeros((3, 8))
Force_Selected_Points_SCM = np.zeros((3, 8))
plt.rc('font',family='Times New Roman')
# ===================================================================
for n in range(2):
    plt.figure(figsize = FigSize)
    font = {'weight': Weight, 'size': fs}
    plt.rc('font', **font)
    for i in range(3):
        # Find the data directory
        dir_tot = dir2 + dir3[i][n] + dir4

        Time = []
        Sinkage = []
        Force = []

        file = open(dir_tot,"r")
        for line in file:
            result = list(map(float, line.split("\t")))
            if len(result) < 4:
                break
            Time.append(result[0])
            Sinkage.append(result[1])
            Force.append(result[2]/1e5)
        file.close()

        x = np.array(Sinkage)
        y = np.array(Force)

        # Fit the curve
        p = P.polynomial.Polynomial.fit(x, y, deg = 15)
        yvals = p(x)

        # Selected points at different sinkage
        for j in range(8):
            sinkage = 0.025 * (j + 1)
            Force_Selected_Points_CRM[0][j] = sinkage
            Force_Selected_Points_CRM[1+n][j] =+ p(sinkage)

        plt.plot(x, yvals, linestyle = LineStyle[i], lw = lw, color = Color[i], label = LableName[i])
        if i==2:
            plt.plot(Force_Selected_Points_CRM[0][:], Force_Selected_Points_CRM[1+n][:], 
                "r1", color = Color[i+1], markersize = ms, fillstyle='none', markeredgewidth = lw, label = LableName[i+1])
        plt.ticklabel_format(axis="y", style="sci", scilimits=(0,0))

    plt.grid(linestyle = '--')
    plt.legend(loc='upper left')
    ax = plt.gca()
    ax.set_xlabel('Sinkage (m)', fontsize = fs, weight = Weight)
    ax.set_ylabel('Force (N)', fontsize = fs, weight = Weight)
    ax.set_xlim([0, 0.22])
    ax.set_ylim([0, 7e3])
    if n == 1:
        ax.set_ylim([0, 2e4])

    for axis in ['top','bottom','left','right']:
        ax.spines[axis].set_linewidth(lw)
    ax.tick_params(width = lw)

    ax.set_xticks([0, 0.05, 0.1, 0.15, 0.2])
    ax.set_xticklabels(['0', '0.05', '0.1', '0.15', '0.2'])
    
    #ax.set_xticks([0.05, 0.1, 0.15, 0.2])
    #ax.set_xticklabels(['0.05', '0.1', '0.15', '0.2'])

    name = "_F_vs_sinkage_diff_push_vel_with_selected_points.png"
    name_png = out_dir + "final_plate_" + plate_size[n] + name
    print("Plot " + name_png)
    plt.savefig(name_png, facecolor = 'w', dpi = DPI)

# Save the selected force at different sinkage
selected_force = out_dir + "plate_selected_force_sinkage_points_dem.txt"
print("Save " + selected_force)
np.savetxt(selected_force, Force_Selected_Points_CRM.transpose(), delimiter = " ")

# ===================================================================
# K_c K_phi and n_exp are obtained from calibration
# Here, compare against SCM (Bekker Wong) 
# This should be plotted after the calibration
for n in range(2):
    K_c = -4957
    K_phi = 235604
    n_exp = 0.883
    A = math.pi * Radii[n] * Radii[n]
    b = Radii[n]
    for i in range(8):
        sinkage = 0.025 * (i + 1)
        F = ( K_c / b + K_phi ) * pow(sinkage, n_exp) * A
        Force_Selected_Points_SCM[0][i] = sinkage
        Force_Selected_Points_SCM[1+n][i] = F

    # Plot a comparision between CRM and SCM (Janosi)
    plt.figure(figsize = FigSize)
    font = {'weight': Weight, 'size': fs}
    plt.rc('font', **font)

    plt.plot(Force_Selected_Points_CRM[0][:], Force_Selected_Points_CRM[1+n][:], 
        'rp-.', lw = lw, markersize = ms, fillstyle='none', markeredgewidth = lw, label = "DEM")
    plt.plot(Force_Selected_Points_SCM[0][:], Force_Selected_Points_SCM[1+n][:], 
        'gd--', lw = lw, markersize = ms+3, fillstyle='none', markeredgewidth = lw, label = "SCM (Bekker Wong)")
    
    plt.grid(linestyle = '--')
    plt.legend(loc='upper left')
    ax = plt.gca()
    ax.set_xlabel('Sinkage (m)', fontsize = fs, weight = Weight)
    ax.set_ylabel('Force (N)', fontsize = fs, weight = Weight)
    ax.set_xlim([0, 0.22])
    ax.set_ylim([0, 7e3])
    if n == 1:
        ax.set_ylim([0, 1.6e4])

    for axis in ['top','bottom','left','right']:
        ax.spines[axis].set_linewidth(lw)
    ax.tick_params(width = lw)

    ax.set_xticks([0, 0.05, 0.1, 0.15, 0.2])
    ax.set_xticklabels(['0', '0.05', '0.1', '0.15', '0.2'])

    plt.ticklabel_format(axis="y", style="sci", scilimits=(0,0))

    name_png = out_dir + "final_plate_" + plate_size[n] + "_F_vs_sinkage_dem_vs_scm.png"
    print("Plot " + name_png)
    plt.savefig(name_png, facecolor = 'w', dpi = DPI)
    plt.show()
########################################################## 
    plt.figure(figsize = FigSize)
    font = {'weight': Weight, 'size': fs}
    plt.rc('font', **font)
    FaceColor = 'w'
    dir1 = "D:/HuaweiMoveData/Users/17908/Desktop/"
    #plt.plot(Force_Selected_Points_SCM[0][:],100.0*abs(Force_Selected_Points_SCM[1+n][:]-Force_Selected_Points_CRM[1+n][:])/Force_Selected_Points_CRM[1+n][:],label='Error Value',linewidth=1,color='b',marker='o',linestyle='--',
    #         markerfacecolor='blue',markersize=10)
    plt.plot(Force_Selected_Points_SCM[0][:], 100.0*abs(Force_Selected_Points_SCM[1+n][:]-Force_Selected_Points_CRM[1+n][:])/Force_Selected_Points_CRM[1+n][:], 
       'bp-.', lw = lw, markersize = ms, fillstyle='none', markeredgewidth = lw, label = str(n*10+20)+"cm plate sinkage test")
    plt.grid(linestyle = '--')
    plt.xlabel('Sinkage(m)', fontsize = fs, weight = Weight)
    plt.ylabel('Relative Error (100%)', fontsize = fs, weight = Weight)
    
    plt.title('')
    plt.legend()
    
    
    
    name='Sinkage_Error'+str(n+1)+'.png'
    plt.savefig(dir1 + name, facecolor = FaceColor, dpi = DPI)

    plt.show()
##########################################################
# Save the selected force at different sinkage
selected_force = out_dir + "plate_selected_force_sinkage_points_scm.txt"
print("Save " + selected_force)
np.savetxt(selected_force, Force_Selected_Points_SCM.transpose(), delimiter = " ")

