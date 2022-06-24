import matplotlib.pyplot as plt
import numpy as np

def plot_voltage(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,0]
    f_1 = data[:,][:,1]
    f_2 = data[:,][:,2]
    f_3 = data[:,][:,3]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='ft') 
    ax.plot(t, f_1, label='fc') 
    ax.plot(t, f_2, label='rt') 
    ax.plot(t, f_3, label='rc')


    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_current(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,0]
    f_1 = data[:,][:,1]
    f_2 = data[:,][:,2]
    f_3 = data[:,][:,3]

    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0, label='ft') 
    ax.plot(t, f_1, label='fc') 
    ax.plot(t, f_2, label='rt') 
    ax.plot(t, f_3, label='rc')


    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()

def plot_total_current(file_name, title):
    data= np.genfromtxt(file_name, delimiter=' ')
    f_0 = data[:,][:,0]
    f_1 = data[:,][:,1]
    f_2 = data[:,][:,2]
    f_3 = data[:,][:,3]
    t = np.linspace(0,f_0.shape[0],f_0.shape[0])

    fig, ax = plt.subplots()
    ax.plot(t, f_0+f_1+f_2+f_3, label='current') 
    ax.set_xlabel('Time [ms]')
    ax.set_ylabel(title)
    ax.set_title(title)
    ax.legend()


plot_voltage('voltage.txt', 'voltage')
plot_current('current.txt', 'current')
plot_total_current('current.txt', 'total_current')

plt.show()
