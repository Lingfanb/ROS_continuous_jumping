import matplotlib.pyplot as plt
import numpy as np

def plot_hip_angle(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,0]
    f_1 = data[:,][:,1]
    f_2 = data[:,][:,2]
    f_3 = data[:,][:,3]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*180/3.14, label='FR') # leg 0
    ax.plot(t, f_1*180/3.14, label='FL') # leg 1
    ax.plot(t, f_2*180/3.14, label='RR') # leg 2
    ax.plot(t, f_3*180/3.14, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('degree')
    ax.set_title(title)
    ax.legend()

def plot_thigh_angle(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,4]
    f_1 = data[:,][:,5]
    f_2 = data[:,][:,6]
    f_3 = data[:,][:,7]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*180/3.14, label='FR') # leg 0
    ax.plot(t, f_1*180/3.14, label='FL') # leg 1
    ax.plot(t, f_2*180/3.14, label='RR') # leg 2
    ax.plot(t, f_3*180/3.14, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('rad')
    ax.set_title(title)
    ax.legend()


def plot_calf_angle(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,8]
    f_1 = data[:,][:,9]
    f_2 = data[:,][:,10]
    f_3 = data[:,][:,11]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0*180/3.14, label='FR') # leg 0
    ax.plot(t, f_1*180/3.14, label='FL') # leg 1
    ax.plot(t, f_2*180/3.14, label='RR') # leg 2
    ax.plot(t, f_3*180/3.14, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('rad')
    ax.set_title(title)
    ax.legend()

#---------------------------------------------------------
def plot_hip_angle_vel(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,12]
    f_1 = data[:,][:,13]
    f_2 = data[:,][:,14]
    f_3 = data[:,][:,15]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_thigh_angle_vel(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,16]
    f_1 = data[:,][:,17]
    f_2 = data[:,][:,18]
    f_3 = data[:,][:,19]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()


def plot_calf_angle_vel(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,20]
    f_1 = data[:,][:,21]
    f_2 = data[:,][:,22]
    f_3 = data[:,][:,23]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

#----------------Actual torque----------------------------------------

def plot_hip_torque(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,24]
    f_1 = data[:,][:,25]
    f_2 = data[:,][:,26]
    f_3 = data[:,][:,27]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_thigh_torque(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,28]
    f_1 = data[:,][:,29]
    f_2 = data[:,][:,30]
    f_3 = data[:,][:,31]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()


def plot_calf_torque(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,32]
    f_1 = data[:,][:,33]
    f_2 = data[:,][:,34]
    f_3 = data[:,][:,35]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

#------------------Command torque----------------------------
def plot_hip_torque_opt(file_name, title): # tau_ff, q_des , q_act, dq_des, dq_act
    data= np.genfromtxt(file_name, delimiter=' ')
    tau_0 = data[:,][:,24]
    tau_1 = data[:,][:,25]
    tau_2 = data[:,][:,26]
    tau_3 = data[:,][:,27]
    t = np.linspace(0,tau_0.shape[0],tau_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, tau_0, label='FR') # leg 0
    ax.plot(t, tau_1, label='FL') # leg 1
    ax.plot(t, tau_2, label='RR') # leg 2
    ax.plot(t, tau_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_thigh_torque_opt(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')

    tau_0 = data[:,][:,28]
    tau_1 = data[:,][:,29]
    tau_2 = data[:,][:,30]
    tau_3 = data[:,][:,31]

    t = np.linspace(0,tau_0.shape[0],tau_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, tau_0, label='FR') # leg 0
    ax.plot(t, tau_1, label='FL') # leg 1
    ax.plot(t, tau_2, label='RR') # leg 2
    ax.plot(t, tau_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_calf_torque_opt(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')

    tau_0 = data[:,][:,32]
    tau_1 = data[:,][:,33]
    tau_2 = data[:,][:,34]
    tau_3 = data[:,][:,35]

    t = np.linspace(0,tau_0.shape[0],tau_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, tau_0, label='FR') # leg 0
    ax.plot(t, tau_1, label='FL') # leg 1
    ax.plot(t, tau_2, label='RR') # leg 2
    ax.plot(t, tau_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

# ------------  plot feedforward torque and command+joint PD torque  -----------------------
#-------------------------------------------------------------------------------------------
def plot_hip_torque_ff(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,0]
    f_1 = data[:,][:,1]
    f_2 = data[:,][:,2]
    f_3 = data[:,][:,3]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_thigh_torque_ff(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,4]
    f_1 = data[:,][:,5]
    f_2 = data[:,][:,6]
    f_3 = data[:,][:,7]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()


def plot_calf_torque_ff(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,8]
    f_1 = data[:,][:,9]
    f_2 = data[:,][:,10]
    f_3 = data[:,][:,11]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_hip_torque_total(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,12]
    f_1 = data[:,][:,13]
    f_2 = data[:,][:,14]
    f_3 = data[:,][:,15]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_thigh_torque_total(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,16]
    f_1 = data[:,][:,17]
    f_2 = data[:,][:,18]
    f_3 = data[:,][:,19]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()


def plot_calf_torque_total(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,20]
    f_1 = data[:,][:,21]
    f_2 = data[:,][:,22]
    f_3 = data[:,][:,23]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='FR') # leg 0
    ax.plot(t, f_1, label='FL') # leg 1
    ax.plot(t, f_2, label='RR') # leg 2
    ax.plot(t, f_3, label='RL') # leg 3

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

#-------------------------------------------------------------
def plot_hip_angle_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,0] # 
    f1_1 = data1[:,][:,1] # 
    f1_2 = data1[:,][:,2] # 
    f1_3 = data1[:,][:,3] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,0] # 
    f2_1 = data2[:,][:,1] # 
    f2_2 = data2[:,][:,2] # 
    f2_3 = data2[:,][:,3] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0, label='FR_act') # 
    ax.plot(t, f1_1, label='FL_act') # 
    ax.plot(t, f1_2, label='RR_act') # 
    ax.plot(t, f1_3, label='RL_act') # 
    ax.plot(t, f2_0, label='FR_des') # 
    ax.plot(t, f2_1, label='FL_des') # 
    ax.plot(t, f2_2, label='RR_des') # 
    ax.plot(t, f2_3, label='RL_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_thigh_angle_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,4] # 
    f1_1 = data1[:,][:,5] # 
    f1_2 = data1[:,][:,6] # 
    f1_3 = data1[:,][:,7] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,4] # 
    f2_1 = data2[:,][:,5] # 
    f2_2 = data2[:,][:,6] # 
    f2_3 = data2[:,][:,7] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0, label='FR_act') # 
    ax.plot(t, f1_1, label='FL_act') # 
    ax.plot(t, f1_2, label='RR_act') # 
    ax.plot(t, f1_3, label='RL_act') # 
    ax.plot(t, f2_0, label='FR_des') # 
    ax.plot(t, f2_1, label='FL_des') # 
    ax.plot(t, f2_2, label='RR_des') # 
    ax.plot(t, f2_3, label='RL_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_calf_angle_compare(file_name_1, file_name_2, title):
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,8] # 
    f1_1 = data1[:,][:,9] # 
    f1_2 = data1[:,][:,10] # 
    f1_3 = data1[:,][:,11] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,8] # 
    f2_1 = data2[:,][:,9] # 
    f2_2 = data2[:,][:,10] # 
    f2_3 = data2[:,][:,11] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f1_0, label='FR_act') # 
    ax.plot(t, f1_1, label='FL_act') # 
    ax.plot(t, f1_2, label='RR_act') # 
    ax.plot(t, f1_3, label='RL_act') # 
    ax.plot(t, f2_0, label='FR_des') # 
    ax.plot(t, f2_1, label='FL_des') # 
    ax.plot(t, f2_2, label='RR_des') # 
    ax.plot(t, f2_3, label='RL_des') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

#-------------------ERRORS---------------------------------------
#-----------------------------------------------------------------
def plot_hip_angle_error(file_name_1, file_name_2, title): # desired - actual
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,0] # 
    f1_1 = data1[:,][:,1] # 
    f1_2 = data1[:,][:,2] # 
    f1_3 = data1[:,][:,3] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,0] # 
    f2_1 = data2[:,][:,1] # 
    f2_2 = data2[:,][:,2] # 
    f2_3 = data2[:,][:,3] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, (f2_0-f1_0)*180/3.14, label='FR') # 
    ax.plot(t, (f2_1-f1_1)*180/3.14, label='FL') # 
    ax.plot(t, (f2_2-f1_2)*180/3.14, label='RR') # 
    ax.plot(t, (f2_3-f1_3)*180/3.14, label='RL') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('degree')
    ax.set_title(title)
    ax.legend()

def plot_thigh_angle_error(file_name_1, file_name_2, title): # desired - actual
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,4] # 
    f1_1 = data1[:,][:,5] # 
    f1_2 = data1[:,][:,6] # 
    f1_3 = data1[:,][:,7] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,4] # 
    f2_1 = data2[:,][:,5] # 
    f2_2 = data2[:,][:,6] # 
    f2_3 = data2[:,][:,7] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, (f2_0-f1_0)*180/3.14, label='FR') # 
    ax.plot(t, (f2_1-f1_1)*180/3.14, label='FL') # 
    ax.plot(t, (f2_2-f1_2)*180/3.14, label='RR') # 
    ax.plot(t, (f2_3-f1_3)*180/3.14, label='RL') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('degree')
    ax.set_title(title)
    ax.legend()

def plot_calf_angle_error(file_name_1, file_name_2, title): # desired - actual
    data1= np.genfromtxt(file_name_1, delimiter=' ') # actual
    f1_0 = data1[:,][:,8] # 
    f1_1 = data1[:,][:,9] # 
    f1_2 = data1[:,][:,10] # 
    f1_3 = data1[:,][:,11] # 
    data2= np.genfromtxt(file_name_2, delimiter=' ') # desired
    f2_0 = data2[:,][:,8] # 
    f2_1 = data2[:,][:,9] # 
    f2_2 = data2[:,][:,10] # 
    f2_3 = data2[:,][:,11] # 

    t = np.linspace(0,f1_0.shape[0],f1_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, (f2_0-f1_0)*180/3.14, label='FR_error') # 
    ax.plot(t, (f2_1-f1_1)*180/3.14, label='FL_error') # 
    ax.plot(t, (f2_2-f1_2)*180/3.14, label='RR_error') # 
    ax.plot(t, (f2_3-f1_3)*180/3.14, label='RL_error') # 

    ax.set_xlabel('Time [ms]')
    ax.set_ylabel('degree')
    ax.set_title(title)
    ax.legend()



#plot_hip_angle('joint_act.txt', 'Actual hip angular position')
#plot_thigh_angle('joint_act.txt', 'Actual thigh angular position')
#plot_calf_angle('joint_act.txt', 'Actual calf angular position')

#plot_hip_angle_error('joint_des.txt','joint_act.txt', 'Hip angular position error')
#plot_thigh_angle_error('joint_des.txt','joint_act.txt', 'Thigh angular position error')
#plot_calf_angle_error('joint_des.txt','joint_act.txt', 'Calf angular position error')

plot_hip_torque('joint_act.txt', 'Actual hip torque')
plot_thigh_torque('joint_act.txt', 'Actual thigh torque')
plot_calf_torque('joint_act.txt', 'Actual calf torque')

plot_hip_torque_total('torque.txt', 'Total hip torque')
plot_thigh_torque_total('torque.txt', 'Total thigh torque')
plot_calf_torque_total('torque.txt', 'Total calf torque')

plot_hip_torque_ff('torque.txt', 'Feedforward hip torque')
plot_thigh_torque_ff('torque.txt', 'Feedforward thigh torque')
plot_calf_torque_ff('torque.txt', 'Feedforward calf torque')

plot_hip_torque_opt('joint_des.txt', 'ref hip torque')
plot_thigh_torque_opt('joint_des.txt', 'ref thigh torque')
plot_calf_torque_opt('joint_des.txt', 'ref calf torque')

plt.show()
