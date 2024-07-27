import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from numpy import polynomial as P
from scipy.interpolate import make_interp_spline
from matplotlib import rcParams
# =====================================================
# Loop for three different wheels: HMMWV, VIPER, Rover
# =====================================================
config = {
    "font.family":'Times New Roman',  # 设置字体类型
    "axes.unicode_minus": False #解决负号无法显示的问题
}
rcParams.update(config)

for n_type in range(1):
    # Choose a wheel type
    wheel_type = "HMMWV_Wheel"
    
    print("Processing data for " + wheel_type)

    # General information of the data and plots
    lw = 2
    fs = 14
    ms = 14
    DPI = 300
    Weight = 'bold'
    fig_size = [8, 6]
    xlim = [0, 15]
    ylim_dbp = [-200, 600]
    ylim_torque = [-20, 100]
    
    legend_pos = 'upper right'
    legend_ncol = 4
    FaceColor = 'w'

    tot_load = 20.0 * 9.81

    # Plot fitted curve?
    plot_fit = False

    # Plot vs time curve every N_p point
    N_p = 10

    # Data directory for DBP and Torque
    dir0 = "/srv/home/whu59/"
    # dir0 = "/home/whu59/research/server/euler"
    #dir1 = "/research/sbel/d_chrono_fsi_granular/chrono_3001/chrono_build/bin/DEMO_OUTPUT/FSI_Single_Wheel_Test"
    #dir2_hmmwv = "/Regular_HMMWV_Tire/dx1cm/31-108kg-slip"
    #dir2_viper = "/Real_VIPER_Wheel/new_dx10mm_h10mm_height15cm/01-108kg-slip"
    dir_singlewheel = "D:/dem_scm_calibration_scripts/Data collection/Singlewheel_Data/hmmwv_wheel_v=100_m=20/slip="
    #dir2 = dir2_hmmwv
    #if wheel_type == "VIPER_Wheel":
        #dir2 = dir2_viper

    slip_hmmwv = [["00", "01", "02", "03", "04", "05", "06", "07", "08"],
                  ["0.0", "0.1", "0.2", "0.3", "0.4", "0.5", "0.6", "0.7", "0.8"],
                 [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]]
    
    slip = slip_hmmwv
    

    dir3_dbp = "/data/DBP.txt"
    dir3_torque = "/data/Torque.txt"
    num_slip = len(slip[0])

    out_dir = "04_plot_single_wheel/"

    DBP_Torque_Mean = np.zeros((3, num_slip))
    DBP_Torque_Mean[0][:num_slip] = slip[2][:num_slip]
    DBP_Torque_Mean_SCM = np.zeros((3, num_slip))
    DBP_Torque_Mean_SCM[0][:num_slip] = slip[2][:num_slip]
    t_start = 5

    # Name of the plot and txt file
    name_png_dbp_crm = "final_hmmwv_wheel_dbp_vs_time_dem.png"
    name_png_torque_crm = "final_hmmwv_wheel_torque_vs_time_dem.png"
    name_txt_dbp_torque_vs_slip_crm = "DBP_Torque_vs_Slip_HMMWV_Wheel_DEM.txt"

    # Name of the plot and txt file
    name_png_dbp_scm = "final_hmmwv_wheel_dbp_vs_time_scm.png"
    name_png_torque_scm = "final_hmmwv_wheel_torque_vs_time_scm.png"
    name_txt_dbp_torque_vs_time_scm = "DBP_Torque_vs_Time_HMMWV_Wheel_SCM.txt"
    name_txt_dbp_torque_vs_slip_scm = "DBP_Torque_vs_Slip_HMMWV_Wheel_SCM.txt"
    
    # ==============================================
    # ==================== DEM =====================
    # ==============================================
    # ==============================================
    # ============= Plot DBP VS time ===============
    print(" DEM: Plot DBP VS time")
    plt.figure(figsize = fig_size)
    font = {'weight': Weight, 'size': fs}
    plt.rc('font', **font)
    for i in range(num_slip):
        # Find the directory of dbp data
        tot_dir = dir_singlewheel + slip[1][i] + dir3_dbp

        file = open(tot_dir,"r")
        Time = []
        DBP = []

        val_tot = 0.0
        ni = 0
        for line in file:
            result = list(map(float, line.split("\t")))
            if len(result) < 4:
                break
            Time.append(result[0])
            DBP.append(result[3]/1e5)
            if result[0] > t_start:
                val_tot = val_tot + result[3]/1e5
                ni = ni + 1
        file.close()

        DBP_Torque_Mean[1][i] = val_tot / ni

        x = np.array(Time)
        y = np.array(DBP)

        if plot_fit == True:
            p = P.polynomial.Polynomial.fit(x, y, deg = 15)
            y = p(x)

        plt.plot(x[::N_p], y[::N_p], linestyle = "--", lw = lw, label = "slip="+slip[1][i])

    plt.grid(linestyle = '--')
    plt.legend(loc = legend_pos, ncol = legend_ncol)
    ax = plt.gca()
    # ax.set_title("No title")
    ax.set_xlabel('Time (s)', fontsize = fs, weight = Weight)
    ax.set_ylabel('DrawBar-Pull (N)', fontsize = fs, weight = Weight)
    ax.set_xlim([xlim[0], xlim[1]])
    ax.set_ylim([ylim_dbp[0], ylim_dbp[1]])

    for axis in ['top','bottom','left','right']:
        ax.spines[axis].set_linewidth(lw)
    ax.tick_params(width = lw)

    plt.savefig(out_dir + name_png_dbp_crm, facecolor = FaceColor, dpi = DPI)
    # plt.show()

    # ==============================================
    # ============ Plot Torque VS time =============
    print(" DEM: Plot Torque VS time")
    plt.figure(figsize = fig_size)
    font = {'weight': Weight, 'size': fs}
    plt.rc('font', **font)
    for i in range(num_slip):
        # Find the directory of torque data
        tot_dir = dir_singlewheel + slip[1][i] + dir3_torque

        file = open(tot_dir,"r")
        Time = []
        Torque = []

        val_tot = 0.0
        ni = 0
        for line in file:
            result = list(map(float, line.split("\t")))
            if len(result) < 4:
                break
            Time.append(result[0])
            Torque.append(-result[5]/1e7)
            if result[0] > t_start:
                val_tot = val_tot -result[5]/1e7
                ni = ni + 1
        file.close()

        DBP_Torque_Mean[2][i] = val_tot / ni

        x = np.array(Time)
        y = np.array(Torque)

        if plot_fit == True:
            p = P.polynomial.Polynomial.fit(x, y, deg = 15)
            y = p(x)

        plt.plot(x[::N_p], y[::N_p], linestyle = "--", lw = lw, label = "slip="+slip[1][i])

    plt.grid(linestyle = '--')
    plt.legend(loc = legend_pos, ncol = legend_ncol)
    ax = plt.gca()
    # ax.set_title("No title")
    ax.set_xlabel('Time (s)', fontsize = fs, weight = Weight)
    ax.set_ylabel('Wheel Torque (Nm)', fontsize = fs, weight = Weight)
    ax.set_xlim([xlim[0], xlim[1]])
    ax.set_ylim([ylim_torque[0], ylim_torque[1]])

    for axis in ['top','bottom','left','right']:
        ax.spines[axis].set_linewidth(lw)
    ax.tick_params(width = lw)

    plt.savefig(out_dir + name_png_torque_crm, facecolor = FaceColor, dpi = DPI)
    # plt.show()

    # Save DBP Torque vs Slip data into a text file
    np.savetxt(out_dir + name_txt_dbp_torque_vs_slip_crm, DBP_Torque_Mean.transpose(), delimiter = " ")

    # ==============================================
    # ==================== SCM =====================
    # ==============================================
    # ==============================================
    # ============= Plot DBP VS time ===============
    print(" SCM: Plot DBP VS time")
    plt.figure(figsize = fig_size)
    font = {'weight': Weight, 'size': fs}
    plt.rc('font', **font)
    for i in range(num_slip):
        # Find the directory of dbp data
        tot_dir = out_dir + name_txt_dbp_torque_vs_time_scm

        Time = []
        DBP = []
        file = open(tot_dir,"r")
        for line in file:
            result = list(map(float, line.split(" ")))
            if len(result) < 4:
                break
            Time.append(result[0])
            #DBP.append(result[1 + i])
            DBP.append(result[1 + i])
        file.close()
#####################################################
        x = np.array(Time)
        y = np.array(DBP)
        
        #x_new = np.linspace(x.min(),x.max(),12000) #300 represents number of points to make between T.min and T.max
        #y_smooth = make_interp_spline(x,y)(x_new)

        if plot_fit == True:
            p = P.polynomial.Polynomial.fit(x, y, deg = 15)
            y = p(x)

        plt.plot(x[::N_p], y[::N_p], linestyle = "--", lw = lw, label = slip[1][i])
        #plt.plot(x_new[::N_p], y_smooth[::N_p], linestyle = "--", lw = lw, label = "slip=0.0")

    plt.grid(linestyle = '--')
    plt.legend(loc = legend_pos, ncol = legend_ncol)
    ax = plt.gca()
    # ax.set_title("No title")
    ax.set_xlabel('Time (s)', fontsize = fs, weight = Weight)
    ax.set_ylabel('DrawBar-Pull (N)', fontsize = fs, weight = Weight)
    ax.set_xlim([xlim[0], xlim[1]])
    ax.set_ylim([ylim_dbp[0], ylim_dbp[1]])

    for axis in ['top','bottom','left','right']:
        ax.spines[axis].set_linewidth(lw)
    ax.tick_params(width = lw)

    plt.savefig(out_dir + name_png_dbp_scm, facecolor = FaceColor, dpi = DPI)
    #plt.show()

    # ==============================================
    # ============= Plot Torque VS time ============
    print(" SCM: Plot Torque VS time")
    plt.figure(figsize = fig_size)
    font = {'weight': Weight, 'size': fs}
    plt.rc('font', **font)
    for i in range(num_slip):
        # Find the directory of dbp data
        tot_dir = out_dir + name_txt_dbp_torque_vs_time_scm

        Time = []
        Torque = []
        file = open(tot_dir,"r")
        for line in file:
            result = list(map(float, line.split(" ")))
            if len(result) < 4:
                break
            Time.append(result[0])
            Torque.append(result[i + num_slip + 1])
            #Torque.append(result[i + num_slip + 9])
        file.close()
#####################################################################
        x = np.array(Time)
        y = np.array(Torque)
        
        #x_new = np.linspace(x.min(),x.max(),10000) #300 represents number of points to make between T.min and T.max
        #y_smooth = make_interp_spline(x,y)(x_new)

        if plot_fit == True:
            p = P.polynomial.Polynomial.fit(x, y, deg = 15)
            y = p(x)

        plt.plot(x[::N_p], y[::N_p], linestyle = "--", lw = lw, label = slip[1][i])
        #plt.plot(x_new[::N_p], y_smooth[::N_p], linestyle = "--", lw = lw, label = "slip=0.0")
    plt.grid(linestyle = '--')
    plt.legend(loc = legend_pos, ncol = legend_ncol)
    ax = plt.gca()
    # ax.set_title("No title")
    ax.set_xlabel('Time (s)', fontsize = fs, weight = Weight)
    ax.set_ylabel('Wheel Torque (Nm)', fontsize = fs, weight = Weight)
    ax.set_xlim([xlim[0], xlim[1]])
    ax.set_ylim([ylim_torque[0], ylim_torque[1]])

    for axis in ['top','bottom','left','right']:
        ax.spines[axis].set_linewidth(lw)
    ax.tick_params(width = lw)

    plt.savefig(out_dir + name_png_torque_scm, facecolor = FaceColor, dpi = DPI)
    #plt.show()


    # ==============================================
    # ================= DEM VS SCM =================
    # ==============================================
    # ==============================================
    # ============== DBP Torque vs Slip ============
    tot_dir = out_dir + name_txt_dbp_torque_vs_slip_scm
    nl = 0
    file = open(tot_dir,"r")
    for line in file:
        result = list(map(float, line.split(" ")))
        DBP_Torque_Mean_SCM[0][nl] = result[0]
        DBP_Torque_Mean_SCM[1][nl] = result[1]
        DBP_Torque_Mean_SCM[2][nl] = result[2]
        nl = nl + 1
    file.close()

    # Loop for DBP, Torque and Slope
    for k in range(3):
        if k == 0:
            print(" DEM VS SCM: Plot DBP VS Slip")
        if k == 1:
            print(" DEM VS SCM: Plot Torque VS Slip")
        if k == 2:
            print(" DEM VS SCM: Plot Slope VS Slip")
        plt.figure(figsize = fig_size)
        font = {'weight': Weight, 'size': fs}
        plt.rc('font', **font)

        x_dem = DBP_Torque_Mean[0][:]
        x_scm = DBP_Torque_Mean_SCM[0][:]

        y_dem = DBP_Torque_Mean[1][:]
        y_scm = DBP_Torque_Mean_SCM[1][:]
        if k == 1:
            y_dem = DBP_Torque_Mean[2][:]
            y_scm = DBP_Torque_Mean_SCM[2][:]
        if k == 2: 
            y_dem = 180 / math.pi * np.arctan(1.0 / tot_load * DBP_Torque_Mean[1][:])
            y_scm = 180 / math.pi * np.arctan(1.0 / tot_load * DBP_Torque_Mean_SCM[1][:])
        plt.plot(x_dem, y_dem, 'rs--', lw = lw, markersize = ms, label = "DEM")
        plt.plot(x_scm, y_scm, 'b*-.', lw = lw, markersize = ms, label = "SCM")

        plt.grid(linestyle = '--')
        plt.legend(loc='upper left')
        ax = plt.gca()

        # ax.set_title("No title")

        ax.set_xlabel('Slip', fontsize = fs, weight = Weight)
        if k == 0:
            ax.set_ylabel('DrawBar-Pull (N)', fontsize = fs, weight = Weight)
        if k == 1:
            ax.set_ylabel('Wheel Torque (Nm)', fontsize = fs, weight = Weight)
        if k == 2:
            ax.set_ylabel('Slope (deg)', fontsize = fs, weight = Weight)
        
        # Set xlim and ylim
        ax.set_xlim([-0.1, 0.9])
        if k == 0:
            ax.set_ylim([-30, 100])
        if k == 1:
            ax.set_ylim([-5, 55])


        if k == 2:
            ax.set_ylim([-10, 30])

        for axis in ['top','bottom','left','right']:
            ax.spines[axis].set_linewidth(lw)
        ax.tick_params(width = lw)

        # Set the output png name
        name_base = "final_hmmwv_wheel"

        name = name_base + "_dbp_vs_slip_dem_vs_scm.png"
        if k == 1:
            name = name_base + "_torque_vs_slip_dem_vs_scm.png"
        if k == 2:
            name = name_base + "_slope_vs_slip_dem_vs_scm.png"
        plt.savefig(out_dir + name, facecolor = FaceColor, dpi = DPI)
        # plt.show()
