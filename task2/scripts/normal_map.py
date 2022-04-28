import cv2
import numpy as np
from matplotlib import pyplot as plt


# Class for storing derivative magnitudes and derivative angles
class MagnitudeAngle:
    magnitudes = [[]]
    angles = [[]]

    def __init__(self, magnitudes, angles):
        self.magnitudes = magnitudes
        self.angles = angles


# Function gradient_magnitude
def gradient_magnitude(image, size, sigma):
    """
    Function that accepts grayscale image and returns both derivative magnitudes
    and derivative angles. Magnitude is calculated as 
    m(x, y) = (Ix(x, y)^2 + Iy(x, y)^2 )^1/2 and angles are calculated as
    and(x, y) = arctan(Iy(x, y) / Ix(x, y)).
    """

    kernel = np.arange(-size, size + 1)
    gauss = np.array([gauss_kernel(kernel, sigma)])
    gaussdx = np.array([gaussdx_kernel(kernel, sigma)])

    image_dx = gauss_filter(image, np.transpose(gauss))
    image_dx = gauss_filter(image_dx, gaussdx)
    image_dy = gauss_filter(image, gauss)
    image_dy = gauss_filter(image_dy, np.transpose(gaussdx))

    # Derivative magnitudes
    magnitudes = np.sqrt(np.square(image_dx) + np.square(image_dy))

    # Derivative angles
    angles = np.arctan2(image_dy, image_dx)

    return MagnitudeAngle(magnitudes, angles)


# Function gauss_filter
def gauss_filter(signal, kernel):
    """
    Function that accepts two Gaussian filters and applies it to a 2D signal. 
    Gaussian kernel is used in one dimension only, then the result is 
    convoluted. The function should return filtered 2D signal.
    """

    filter = cv2.filter2D(signal, -1, np.flip(kernel))

    return filter


# Function gaussdx_kernel
def gaussdx_kernel(kernel_size, sigma):
    """
    Function that uses desired kernel and the parameter sigma. The 
    parameter sigma defines the shape of the kernel. The function computes
    derivate of 1D Gaussian kernel. The function returns derivated gaussian 
    kernel with desired size and sigma.
    """

    kernel = -(1 * kernel_size /
               (np.sqrt(2 * np.pi) * np.power(sigma, 3))) * np.power(
                   np.e, -(np.square(kernel_size) / (2 * np.square(sigma))))
    kernel /= np.sum(np.abs(kernel))

    return kernel


# Function gauss_kernel
def gauss_kernel(kernel_size, sigma):
    """
    Function that uses desired kernel and the parameter sigma. The 
    parameter sigma defines the shape of the kernel. The function returns 
    gaussian kernel with desired size and sigma.
    """

    kernel = (1 / (np.sqrt(2 * np.pi) * sigma)) * np.power(
        np.e, -(np.square(kernel_size) / (2 * np.square(sigma))))
    kernel /= np.sum(kernel)

    return kernel


def calc_normal_map():
    image = cv2.imread(
        "/home/mokot/FRI/RazvojInteligentnihSistemov/ROS/src/task2/maps/map.pgm"
    )
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    image = image.astype(float)

    # Calculate derivative magnitudes and derivative angles
    magnitudeAngle = gradient_magnitude(image, 7, 0.5)
    return magnitudeAngle.angles, magnitudeAngle.magnitudes


plt.figure(figsize=(15, 12))
plt.imshow(calc_normal_map()[0], cmap='gray')
plt.imshow(calc_normal_map()[1], cmap='gray')
plt.show()
# np.save("/home/mokot/FRI/RazvojInteligentnihSistemov/ROS/src/task2/maps/normal_map.npy", calc_normal_map())