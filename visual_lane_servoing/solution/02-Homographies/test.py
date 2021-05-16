import matplotlib
#matplotlib.use('WebAgg')
from matplotlib import pyplot as plt
import numpy as np
import cv2

# TODO: Running this cell will bring up a figure displaying the two above images of the scene. When you click a point in the left image,
#.      the projected point in the right image will be rendered according to the homography induced by the ground plane.
#.      Compare the accuracy of the correspondences for points that lie on the ground plane to points elsewhere in the environment.
imgl = cv2.imread('/mnt/m2ssd1/drivesim/duckietown/mooc-exercises/visual_lane_servoing/solution/images/homography/bt.000.png', 0)
imgr = cv2.imread('/mnt/m2ssd1/drivesim/duckietown/mooc-exercises/visual_lane_servoing/solution/images/homography/bt.002.png', 0)

H = np.array([[0.907503833504229, -0.116496578881938, 30.8471918181923],[0.00308072860216055, 0.828815989469247, 16.0448537015201],[-1.74015013507422e-05, -0.000441721032603193, 1]])

fig = plt.figure(figsize = (20,20))
ax1 = fig.add_subplot(1,2,1)
ax1.imshow(imgl, cmap = 'gray')
ax1.set_title('Source Image');
ax2 = fig.add_subplot(1,2,2)
ax2.imshow(imgr,cmap = 'gray')
ax2.set_title('Target Image');

def onclick(event):
    # Apply the homography
    x = np.array([[event.xdata, event.ydata, 1]]).transpose()
    xprime = H.dot(x)
    xprime = xprime/xprime[2]
    
    # Visualize the selected and projected points
    ax1.plot(x[0], x[1], 'rx')
    ax2.plot(xprime[0], xprime[1], 'rx')
    plt.show()

cid = fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()

print('Done')