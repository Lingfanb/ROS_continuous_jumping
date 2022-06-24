import matplotlib.pyplot as plt
import numpy as np

def plot_COM_vel_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,0]
    f_1 = data[:,][:,1]
    f_2 = data[:,][:,2]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='vx') 
    ax.plot(t, f_1, label='vy') 
    ax.plot(t, f_2, label='vz') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_COM_vel_des(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,0]
    f_1 = data[:,][:,1]
    f_2 = data[:,][:,2]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='vx') 
    ax.plot(t, f_1, label='vy') 
    ax.plot(t, f_2, label='vz') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_COM_pos_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,3]
    f_1 = data[:,][:,4]
    f_2 = data[:,][:,5]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='px') 
    ax.plot(t, f_1, label='py') 
    ax.plot(t, f_2, label='pz') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_COM_pos_des(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,3]
    f_1 = data[:,][:,4]
    f_2 = data[:,][:,5]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='px') 
    ax.plot(t, f_1, label='py') 
    ax.plot(t, f_2, label='pz') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_COM_rpy_act(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,6]
    f_1 = data[:,][:,7]
    f_2 = data[:,][:,8]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*180/3.14, label='roll') 
    ax.plot(t, f_1*180/3.14, label='pitch') 
    ax.plot(t, f_2*180/3.14, label='yaw') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_COM_rpy_des(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,6]
    f_1 = data[:,][:,7]
    f_2 = data[:,][:,8]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='roll') 
    ax.plot(t, f_1, label='pitch') 
    ax.plot(t, f_2, label='yaw') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()


def plot_feedforwardForce(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,9] # FR leg
    f_1 = data[:,][:,10] # FL leg
    f_2 = data[:,][:,11] # RR leg
    f_3 = data[:,][:,12] # RL leg

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') 
    ax.plot(t, f_1, label='FL') 
    ax.plot(t, f_2, label='RR') 
    ax.plot(t, f_3, label='RL') 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_COM_pos_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,3] # 
    f1_1 = data1[:,][:,4] # 
    f1_2 = data1[:,][:,5] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,3] # 
    f2_1 = data2[:,][:,4] # 
    f2_2 = data2[:,][:,5] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0, label='px_act') # 
    ax.plot(t, f1_1, label='py_act') # 
    ax.plot(t, f1_2, label='pz_act') # 
    ax.plot(t, f2_0, label='px_des') # 
    ax.plot(t, f2_1, label='py_des') # 
    ax.plot(t, f2_2, label='pz_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_COM_vel_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,0] # 
    f1_1 = data1[:,][:,1] # 
    f1_2 = data1[:,][:,2] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,0] # 
    f2_1 = data2[:,][:,1] # 
    f2_2 = data2[:,][:,2] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0, label='vx_act') # 
    ax.plot(t, f1_1, label='vy_act') # 
    ax.plot(t, f1_2, label='vz_act') # 
    ax.plot(t, f2_0, label='vx_des') # 
    ax.plot(t, f2_1, label='vy_des') # 
    ax.plot(t, f2_2, label='vz_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_COM_rpy_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,6] # 
    f1_1 = data1[:,][:,7] # 
    f1_2 = data1[:,][:,8] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,6] # 
    f2_1 = data2[:,][:,7] # 
    f2_2 = data2[:,][:,8] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0*180/3.14, label='r_act') # 
    ax.plot(t, f1_1*180/3.14, label='p_act') # 
    ax.plot(t, f1_2*180/3.14, label='y_act') # 
    ax.plot(t, f2_0*180/3.14, label='r_des') # 
    ax.plot(t, f2_1*180/3.14, label='p_des') # 
    ax.plot(t, f2_2*180/3.14, label='y_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('degree')
    ax.set_title(title)
    ax.legend()


#plot_COM_vel_act('pos.txt', 'COM_vel_act')
plot_COM_rpy_act('pos.txt', 'COM_rpy_act')
plot_COM_pos_act('pos.txt', 'COM_pos_act')
plot_feedforwardForce('pos.txt', 'feedforwardForce')

#plot_COM_vel_des('com_des.txt', 'COM_vel_des')
#plot_COM_pos_des('com_des.txt', 'COM_pos_des')
#plot_COM_rpy_des('com_des.txt', 'RPY_des')

plot_COM_pos_compare('com_act.txt','com_des.txt','COM comparison')
plot_COM_rpy_compare('com_act.txt','com_des.txt','RPY comparison')
plot_COM_rpy_act('pos.txt','RPY act')

plt.show()
