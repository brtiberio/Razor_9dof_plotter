import matplotlib
import sys
import os
from transforms3d.taitbryan import euler2mat
import time
from RazorIMU_interface_python.razorIMU import RazorIMU
import logging
import argparse
import numpy as np

if sys.version_info.major == 3:
    import queue

    matplotlib.use('Qt5Agg')
else:
    import Queue as queue

    matplotlib.use('Qt4Agg')

matplotlib.rcParams['toolbar'] = 'None'
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt


def sphere(n=20, radius=1):
    """
    Create a sphere with radius of 1 unit interpolated using 20 points

    Args:
        n: interpolation points
        radius: radius of sphere
    Return:
        tuple: A tuple containing vector interpolation in x, y, z axes
    """
    if not isinstance(n, int):
        print("Input value n must be a integer number: n = {}".format(n))
        return
    if not n > 0:
        print("Input value n must be positive: n = {}".format(n))
        return

    # Make data
    u = np.linspace(0, 2 * np.pi, n)
    v = np.linspace(0, np.pi, n)
    x = radius * np.outer(np.cos(u), np.sin(v))
    y = radius * np.outer(np.sin(u), np.sin(v))
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v))
    return x, y, z


pi = np.pi
cos = np.cos
sin = np.sin
to_rad = pi / 180.0
to_deg = 180.0 / pi
figClosed = False

# create a light sphere for easier understanding rotations
# It looks better if the lines are lighter
lightGrey = (0.8, 0.8, 0.8)
# other colors similar to matlab
blueColor = (0, 0.4470, 0.7410)
redColor = (0.8500, 0.3250, 0.0980)
yellowColor = (0.9290, 0.6940, 0.1250)


class Plotter:

    def __init__(self):
        self.x_body = np.array([1, 0, 0]).reshape((3, 1))
        self.y_body = np.array([0, 1, 0]).reshape((3, 1))
        self.z_body = np.array([0, 0, 1]).reshape((3, 1))
        self.rotMatrix = []
        # create a circumference for creating reference planes
        self.cirx = cos(np.arange(0, 2.0 * pi, 0.01))
        self.ciry = sin(np.arange(0, 2.0 * pi, 0.01))
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        # plot a wire frame sphere
        x, y, z = sphere(20)
        # Plot the surface
        self.ax.plot_wireframe(x, y, z,
                               color=lightGrey,
                               linestyle='-.',
                               linewidth=0.5)
        # plot reference plane circles
        self.ax.plot3D(self.cirx, self.ciry, 0,
                       color='black', linestyle='--')
        self.ax.plot3D(np.zeros(self.cirx.size), self.cirx, self.ciry,
                       color='black', linestyle='--')
        # set axes limits and aspect
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_aspect("equal")
        self.ax.set_axis_off()

        # add text annotations
        self.ax.text(0, 1, 0, 'N', weight='bold', fontsize=12)
        self.ax.text(1, 0, 0, 'E', weight='bold', fontsize=12)
        self.ax.text(0, -1, 0, 'S', weight='bold', fontsize=12)
        self.ax.text(-1, 0, 0, 'W', weight='bold', fontsize=12)

        # add title and legend as subtitle
        plt.suptitle('Body frame orientation', fontsize=16)
        self.fig.text(0.4, 0.90, "X-axis", ha="center", va="bottom",
                      size="medium", color=blueColor)
        self.fig.text(0.5, 0.90, "Y-axis", ha="center", va="bottom",
                      size="medium", color=redColor)
        self.fig.text(0.6, 0.90, "Z-axis", ha="center", va="bottom",
                      size="medium", color=yellowColor)

        # plot quiver for each axis representing unit vector
        self.x, = self.ax.plot3D([0, 1], [0, 0], [0, 0], color=blueColor)
        self.y, = self.ax.plot3D([0, 0], [0, 1], [0, 0], color=redColor)
        self.z, = self.ax.plot3D([0, 0], [0, 0], [0, 1], color=yellowColor)

        self.phiText = self.fig.text(0.2, 0.10, "phi=+000.00", ha="center",
                                     va="bottom", size="medium", color=blueColor)
        self.thetaText = self.fig.text(0.5, 0.10, "theta=+000.00", ha="center",
                                       va="bottom", size="medium", color=redColor)
        self.psiText = self.fig.text(0.8, 0.10, "psi=+000.00", ha="center",
                                     va="bottom", size="medium", color=yellowColor)
        plt.tight_layout()

    def update(self, psi, theta, phi):
        # calculate rotation matrix
        self.rotMatrix = euler2mat(psi, theta, phi)
        # calculate new axes vectors
        new_x = np.dot(self.rotMatrix, self.x_body)
        new_y = np.dot(self.rotMatrix, self.y_body)
        new_z = np.dot(self.rotMatrix, self.z_body)
        # update x axis
        self.x.set_xdata(np.array([0, new_x[0]]))
        self.x.set_ydata(np.array([0, new_x[1]]))
        self.x.set_3d_properties(np.array([0, new_x[2]]))
        # update y axis
        self.y.set_xdata(np.array([0, new_y[0]]))
        self.y.set_ydata(np.array([0, new_y[1]]))
        self.y.set_3d_properties(np.array([0, new_y[2]]))
        # update z axis
        self.z.set_xdata(np.array([0, new_z[0]]))
        self.z.set_ydata(np.array([0, new_z[1]]))
        self.z.set_3d_properties(np.array([0, new_z[2]]))

        self.psiText.set_text('psi={:+07.2f}'.format(psi * to_deg))
        self.thetaText.set_text('theta={:+07.2f}'.format(theta * to_deg))
        self.phiText.set_text('phi={:+07.2f}'.format(phi * to_deg))
        return


def main():
    def handle_close(evt):
        global figClosed
        razor.shutdown()
        print('Closed Figure!')
        figClosed = True
        return

    def save_razor_data():
        fileFP.write('{0:5d},{1},{2},{3:.2f},{4:.2f},{5:.2f},{6:.2f},'
                     '{7:.2f},{8:.2f},{9:.2f},{10:.2f},{11:.2f},'
                     '{12:.2f},{13:.2f},{14:.2f}'
                     '\n'.format(newData['Index'],
                                 newData['Time'],
                                 newData['ID'],
                                 newData['Acc'][0],
                                 newData['Acc'][1],
                                 newData['Acc'][2],
                                 newData['Gyro'][0],
                                 newData['Gyro'][1],
                                 newData['Gyro'][2],
                                 newData['Mag'][0],
                                 newData['Mag'][1],
                                 newData['Mag'][2],
                                 newData['euler'][0],
                                 newData['euler'][1],
                                 newData['euler'][2]))
        fileFP.flush()
        return

    ############################################################################
    #
    # Start of main part
    #
    ############################################################################

    parser = argparse.ArgumentParser(usage="usage: %prog [options] args")
    parser.add_argument("--port", dest="port", default="/dev/ttyUSB0")
    parser.add_argument("--folder", dest="folder", default="test1")
    parser.add_argument("--file", dest="file", default="razor.csv")
    parser.add_argument("--baud", dest="baud", default=500000)
    parser.add_argument("--frameStep", dest="frameStep", default=10)
    parser.add_argument("--useEuler", dest="use_euler", default=True)
    args = parser.parse_args()

    use_euler = args.use_euler
    filename = args.file
    port = args.port
    baud = args.baud

    # check dir to save data
    current_dir = os.getcwd()
    current_dir = current_dir + "/data/" + args.folder
    if not os.path.exists(current_dir):
        os.makedirs(current_dir)

    os.chdir(current_dir)
    logging.basicConfig(level=logging.INFO,
                        format='[%(asctime)s] [%(threadName)-10s] %(levelname)-8s %(message)s',
                        stream=sys.stdout)

    # create a queue to receive values
    dataFIFO = queue.Queue()

    fileFP = open(filename, 'w')
    fileFP.write("Index,Time,ID,accx,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz,yaw,pitch,roll\n")
    fileFP.flush()
    # instantiate a class object
    razor = RazorIMU("Razor1")

    plotter = Plotter()
    plotter.fig.canvas.mpl_connect('close_event', handle_close)
    plt.show(block=False)
    plotter.fig.canvas.draw()

    # begin
    if razor.begin(dataFIFO, com_port=port, baud_rate=baud) != 1:
        logging.info("Not able to begin device properly... check logfile")
        return

    num_plots = 0
    while not figClosed:
        if dataFIFO.empty() is False:
            newData = dataFIFO.get()
            # euler = [phi theta psi] = [roll pitch yaw]
            if use_euler:
                phi = newData['euler'][0] * to_rad
                theta = newData['euler'][1] * to_rad
                psi = newData['euler'][2] * to_rad
            else:
                pass

            # print("{0:.02f},{1:.02f},{2:.02f}\n".format(psi, theta, phi))
            # do not update all frames, only a few to avoid delay.
            if num_plots % args.frameStep == 0:
                plotter.update(psi, theta, phi)
                plotter.fig.canvas.draw()
                plotter.fig.canvas.flush_events()
            num_plots += 1
            save_razor_data()
        else:
            time.sleep(1 / 200.0)

    fileFP.close()
    # exit now
    logging.info('Exiting now')
    logging.shutdown()
    return


if __name__ == '__main__':
    main()
