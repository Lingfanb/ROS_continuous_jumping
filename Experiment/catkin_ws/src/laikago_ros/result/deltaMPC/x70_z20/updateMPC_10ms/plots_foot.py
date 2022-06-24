import matplotlib.pyplot as plt
import numpy as np

def plot_pos_x_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,0] # FR
    f_1 = data[:,][:,3] # FL
    f_2 = data[:,][:,6] # RR
    f_3 = data[:,][:,9] # RL

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm')
    ax.set_title(title)
    ax.legend()

def plot_pos_y_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,1]
    f_1 = data[:,][:,4]
    f_2 = data[:,][:,7]
    f_3 = data[:,][:,10]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm')
    ax.set_title(title)
    ax.legend()


def plot_pos_z_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,2]
    f_1 = data[:,][:,5]
    f_2 = data[:,][:,8]
    f_3 = data[:,][:,11]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm')
    ax.set_title(title)
    ax.legend()

#---------------------------------------------------------
def plot_vel_x_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,12] # FR
    f_1 = data[:,][:,15] # FL
    f_2 = data[:,][:,18] # RR
    f_3 = data[:,][:,21] # RL

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()

def plot_vel_y_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,13]
    f_1 = data[:,][:,16]
    f_2 = data[:,][:,19]
    f_3 = data[:,][:,22]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()


def plot_vel_z_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,14]
    f_1 = data[:,][:,17]
    f_2 = data[:,][:,20]
    f_3 = data[:,][:,23]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()

#-------------------------------------------------------------------

def plot_pos_x_des(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,0] # FR
    f_1 = data[:,][:,3] # FL
    f_2 = data[:,][:,6] # RR
    f_3 = data[:,][:,9] # RL

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm')
    ax.set_title(title)
    ax.legend()

def plot_pos_y_des(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,1]
    f_1 = data[:,][:,4]
    f_2 = data[:,][:,7]
    f_3 = data[:,][:,10]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm')
    ax.set_title(title)
    ax.legend()


def plot_pos_z_des(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,2]
    f_1 = data[:,][:,5]
    f_2 = data[:,][:,8]
    f_3 = data[:,][:,11]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm')
    ax.set_title(title)
    ax.legend()

#-------------------------------------------------------------
def plot_vel_x_des(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,12] # FR
    f_1 = data[:,][:,15] # FL
    f_2 = data[:,][:,18] # RR
    f_3 = data[:,][:,21] # RL

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()

def plot_vel_y_des(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,13]
    f_1 = data[:,][:,16]
    f_2 = data[:,][:,19]
    f_3 = data[:,][:,22]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()


def plot_vel_z_des(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,14]
    f_1 = data[:,][:,17]
    f_2 = data[:,][:,20]
    f_3 = data[:,][:,23]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*100, label='FR') # leg 0
    ax.plot(t, f_1*100, label='FL') # leg 1
    ax.plot(t, f_2*100, label='RR') # leg 2
    ax.plot(t, f_3*100, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()
#-------------------------------------------------------------


def plot_pos_x_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,0] # 
    #f1_1 = data1[:,][:,3] # 
    f1_2 = data1[:,][:,6] # 
    #f1_3 = data1[:,][:,9] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,0] # 
    #f2_1 = data2[:,][:,3] # 
    f2_2 = data2[:,][:,6] # 
    #f2_3 = data2[:,][:,9] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0*100, label='FR_act') # 
    #ax.plot(t, f1_1*100, label='FL_act') # 
    ax.plot(t, f1_2*100, label='RR_act') # 
    #ax.plot(t, f1_3*100, label='RL_act') # 
    ax.plot(t, f2_0*100, label='FR_des') # 
    #ax.plot(t, f2_1*100, label='FL_des') # 
    ax.plot(t, f2_2*100, label='RR_des') # 
    #ax.plot(t, f2_3*100, label='RL_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm')
    ax.set_title(title)
    ax.legend()

def plot_pos_y_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,1] # 
    #f1_1 = data1[:,][:,4] # 
    f1_2 = data1[:,][:,7] # 
    #f1_3 = data1[:,][:,10] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,1] # 
    #f2_1 = data2[:,][:,4] # 
    f2_2 = data2[:,][:,7] # 
    #f2_3 = data2[:,][:,10] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0*100, label='FR_act') # 
    #ax.plot(t, f1_1*100, label='FL_act') # 
    ax.plot(t, f1_2*100, label='RR_act') # 
    #ax.plot(t, f1_3*100, label='RL_act') # 
    ax.plot(t, f2_0*100, label='FR_des') # 
    #ax.plot(t, f2_1*100, label='FL_des') # 
    ax.plot(t, f2_2*100, label='RR_des') # 
    #ax.plot(t, f2_3*100, label='RL_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm')
    ax.set_title(title)
    ax.legend()

def plot_pos_z_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,2] # 
    #f1_1 = data1[:,][:,5] # 
    f1_2 = data1[:,][:,8] # 
    #f1_3 = data1[:,][:,11] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,2] # 
    #f2_1 = data2[:,][:,5] # 
    f2_2 = data2[:,][:,8] # 
    #f2_3 = data2[:,][:,11] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0*100, label='FR_act') # 
    #ax.plot(t, f1_1*100, label='FL_act') # 
    ax.plot(t, f1_2*100, label='RR_act') # 
    #ax.plot(t, f1_3*100, label='RL_act') # 
    ax.plot(t, f2_0*100, label='FR_des') # 
    #ax.plot(t, f2_1*100, label='FL_des') # 
    ax.plot(t, f2_2*100, label='RR_des') # 
    #ax.plot(t, f2_3*100, label='RL_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm')
    ax.set_title(title)
    ax.legend()

#---------------------------------------------------------------

def plot_vel_x_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,12] # 
    #f1_1 = data1[:,][:,15] # 
    f1_2 = data1[:,][:,18] # 
    #f1_3 = data1[:,][:,21] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,12] # 
    #f2_1 = data2[:,][:,15] # 
    f2_2 = data2[:,][:,18] # 
    #f2_3 = data2[:,][:,21] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0*100, label='FR_act') # 
    #ax.plot(t, f1_1*100, label='FL_act') # 
    ax.plot(t, f1_2*100, label='RR_act') # 
    #ax.plot(t, f1_3*100, label='RL_act') # 
    ax.plot(t, f2_0*100, label='FR_des') # 
    #ax.plot(t, f2_1*100, label='FL_des') # 
    ax.plot(t, f2_2*100, label='RR_des') # 
    #ax.plot(t, f2_3*100, label='RL_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()

def plot_vel_y_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,13] # 
    #f1_1 = data1[:,][:,16] # 
    f1_2 = data1[:,][:,19] # 
    #f1_3 = data1[:,][:,22] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,13] # 
    #f2_1 = data2[:,][:,16] # 
    f2_2 = data2[:,][:,19] # 
    #f2_3 = data2[:,][:,22] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0*100, label='FR_act') # 
    #ax.plot(t, f1_1*100, label='FL_act') # 
    ax.plot(t, f1_2*100, label='RR_act') # 
    #ax.plot(t, f1_3*100, label='RL_act') # 
    ax.plot(t, f2_0*100, label='FR_des') # 
    #ax.plot(t, f2_1*100, label='FL_des') # 
    ax.plot(t, f2_2*100, label='RR_des') # 
    #ax.plot(t, f2_3*100, label='RL_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()

def plot_vel_z_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,14] # 
    #f1_1 = data1[:,][:,17] # 
    f1_2 = data1[:,][:,20] # 
    #f1_3 = data1[:,][:,23] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,14] # 
    #f2_1 = data2[:,][:,17] # 
    f2_2 = data2[:,][:,20] # 
    #f2_3 = data2[:,][:,23] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0*100, label='FR_act') # 
    #ax.plot(t, f1_1*100, label='FL_act') # 
    ax.plot(t, f1_2*100, label='RR_act') # 
    #ax.plot(t, f1_3*100, label='RL_act') # 
    ax.plot(t, f2_0*100, label='FR_des') # 
    #ax.plot(t, f2_1*100, label='FL_des') # 
    ax.plot(t, f2_2*100, label='RR_des') # 
    #ax.plot(t, f2_3*100, label='RL_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()

#-------------------ERRORS---------------------------------------
#-----------------------------------------------------------------
def plot_pos_x_error(file_name_1, file_name_2, title): # desired - actual
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,0] # 
    f1_1 = data1[:,][:,3] # 
    f1_2 = data1[:,][:,6] # 
    f1_3 = data1[:,][:,9] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,0] # 
    f2_1 = data2[:,][:,3] # 
    f2_2 = data2[:,][:,6] # 
    f2_3 = data2[:,][:,9] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, (f2_0-f1_0)*100, label='FR') # 
    ax.plot(t, (f2_1-f1_1)*100, label='FL') # 
    ax.plot(t, (f2_2-f1_2)*100, label='RR') # 
    ax.plot(t, (f2_3-f1_3)*100, label='RL') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm')
    ax.set_title(title)
    ax.legend()

def plot_pos_y_error(file_name_1, file_name_2, title): # desired - actual
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,1] # 
    f1_1 = data1[:,][:,4] # 
    f1_2 = data1[:,][:,7] # 
    f1_3 = data1[:,][:,10] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,1] # 
    f2_1 = data2[:,][:,4] # 
    f2_2 = data2[:,][:,7] # 
    f2_3 = data2[:,][:,10] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, (f2_0-f1_0)*100, label='FR') # 
    ax.plot(t, (f2_1-f1_1)*100, label='FL') # 
    ax.plot(t, (f2_2-f1_2)*100, label='RR') # 
    ax.plot(t, (f2_3-f1_3)*100, label='RL') #  

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('Error [cm]')
    ax.set_title(title)
    ax.legend()

def plot_pos_z_error(file_name_1, file_name_2, title): # desired - actual
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,2] # 
    f1_1 = data1[:,][:,5] # 
    f1_2 = data1[:,][:,8] # 
    f1_3 = data1[:,][:,11] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,2] # 
    f2_1 = data2[:,][:,5] # 
    f2_2 = data2[:,][:,8] # 
    f2_3 = data2[:,][:,11] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, (f2_0-f1_0)*100, label='FR') # 
    ax.plot(t, (f2_1-f1_1)*100, label='FL') # 
    ax.plot(t, (f2_2-f1_2)*100, label='RR') # 
    ax.plot(t, (f2_3-f1_3)*100, label='RL') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('Error [cm]')
    ax.set_title(title)
    ax.legend()

#------------------------------------------------------------
def plot_vel_x_error(file_name_1, file_name_2, title): # desired - actual
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,12] # 
    #f1_1 = data1[:,][:,15] # 
    f1_2 = data1[:,][:,18] # 
    #f1_3 = data1[:,][:,21] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,12] # 
    #f2_1 = data2[:,][:,15] # 
    f2_2 = data2[:,][:,18] # 
    #f2_3 = data2[:,][:,21] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, (f2_0-f1_0)*100, label='FR_error') # 
    #ax.plot(t, (f2_1-f1_1)*100, label='FL_error') # 
    ax.plot(t, (f2_2-f1_2)*100, label='RR_error') # 
    #ax.plot(t, (f2_3-f1_3)*100, label='RL_error') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()

def plot_vel_y_error(file_name_1, file_name_2, title): # desired - actual
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,13] # 
    #f1_1 = data1[:,][:,16] # 
    f1_2 = data1[:,][:,19] # 
    #f1_3 = data1[:,][:,21] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,13] # 
    #f2_1 = data2[:,][:,16] # 
    f2_2 = data2[:,][:,19] # 
    #f2_3 = data2[:,][:,21] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, (f2_0-f1_0)*100, label='FR_error') # 
    #ax.plot(t, (f2_1-f1_1)*100, label='FL_error') # 
    ax.plot(t, (f2_2-f1_2)*100, label='RR_error') # 
    #ax.plot(t, (f2_3-f1_3)*100, label='RL_error') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()

def plot_vel_z_error(file_name_1, file_name_2, title): # desired - actual
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,14] # 
    #f1_1 = data1[:,][:,17] # 
    f1_2 = data1[:,][:,20] # 
    #f1_3 = data1[:,][:,23] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,14] # 
    #f2_1 = data2[:,][:,17] # 
    f2_2 = data2[:,][:,20] # 
    #f2_3 = data2[:,][:,23] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, (f2_0-f1_0)*100, label='FR_error') # 
    #ax.plot(t, (f2_1-f1_1)*100, label='FL_error') # 
    ax.plot(t, (f2_2-f1_2)*100, label='RR_error') # 
    #ax.plot(t, (f2_3-f1_3)*100, label='RL_error') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('cm/s')
    ax.set_title(title)
    ax.legend()

#-------------------------------------------------------------------------


#plot_pos_x_act('foot_act.txt', 'Actual foot position x')
#plot_pos_y_act('foot_act.txt', 'Actual foot position y')
#plot_pos_z_act('foot_act.txt', 'Actual foot position z')

plot_pos_x_error('foot_act.txt','foot_des.txt','Foot position x error')
plot_pos_y_error('foot_act.txt','foot_des.txt','Foot position y error')
plot_pos_z_error('foot_act.txt','foot_des.txt','Foot position z error')

#plot_vel_x_error('foot_act.txt','foot_des.txt','Foot velocity x error')
#plot_vel_y_error('foot_act.txt','foot_des.txt','Foot velocity y error')
#plot_vel_z_error('foot_act.txt','foot_des.txt','Foot velocity z error')

#plot_pos_x_compare('foot_act.txt','foot_des.txt','Foot position x compare')
#plot_pos_y_compare('foot_act.txt','foot_des.txt','Foot position y compare')
#plot_pos_z_compare('foot_act.txt','foot_des.txt','Foot position z compare')

plot_vel_x_compare('foot_act.txt','foot_des.txt','Foot velocity x compare')
plot_vel_y_compare('foot_act.txt','foot_des.txt','Foot velocity y compare')
plot_vel_z_compare('foot_act.txt','foot_des.txt','Foot velocity z compare')



plt.show()
