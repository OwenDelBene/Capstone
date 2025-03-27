import numpy as np
import matplotlib.pyplot as plt


def file2np(fname):
    raw = open(fname, 'r').read()
    lines = raw.split('\n')
    return np.array([line.split(',')[:-1] for line in lines[:-1]], dtype=float)
    
    


if __name__ == "__main__":

    ekf_eul = file2np('ekf_eul.out')
    true_eul = file2np('true_eul.out')
    ekf_cov = file2np('ekf_cov.out')
    noise_eul = file2np('noise_eul.out')
    sensor = file2np('sensor.out')
    qerr = file2np('qerr.out')
    t = np.linspace(0, 1, ekf_eul.shape[0])
    
    plt.plot(t, np.degrees(ekf_eul[:, 0]), label='ekf')
    plt.plot(t, np.degrees(noise_eul[:, 0]), label='true')
    plt.title('ekf vs true roll')
    plt.legend()
    plt.show()
    
    plt.plot(t, np.degrees(ekf_eul[:, 1]), label='ekf')
    plt.plot(t, np.degrees(noise_eul[:, 1]), label='true')
    plt.title('ekf vs true pitch')
    plt.legend()
    plt.show()
   


    plt.plot(t, ekf_cov[:, 0], label='x')
    plt.plot(t, ekf_cov[:, 1], label='y')
    plt.plot(t, ekf_cov[:, 2], label='z')
    plt.title('ekf_cov ax, ay, az')
    plt.legend()
    plt.show()
    
    plt.plot(t, sensor[:, 0], label='ax')
    plt.plot(t, sensor[:, 1], label='ay')
    plt.plot(t, sensor[:, 2], label='az')
    plt.title('accel xx, yy, zz')
    plt.legend()
    plt.show()
    
    plt.plot(t, sensor[:, 3], label='mx')
    plt.plot(t, sensor[:, 4], label='my')
    plt.plot(t, sensor[:, 5], label='mz')
    plt.title('mag xx, yy, zz')
    plt.legend()
    plt.show()
    

    plt.plot(t, qerr[:, 0], label='w')
    plt.plot(t, qerr[:, 1], label='x')
    plt.plot(t, qerr[:, 2], label='y')
    plt.plot(t, qerr[:, 3], label='z')
    plt.title('quat err')
    plt.legend()
    plt.show()
    
    mag = [np.linalg.norm(i) for i in qerr ]
    plt.plot(t, mag, label='mag')
    plt.title('quat err mag')
    plt.legend()
    plt.show()
