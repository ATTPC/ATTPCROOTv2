import math
import numpy as np

def convert_to_spherical(x, y, z):
    r = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arccos(z / r)
    phi = np.arctan2(y, x)
    return theta, phi

def adjust_distance(old_theta, old_phi, old_r, new_r):
    x = old_r * np.sin(old_theta) * np.cos(old_phi)
    y = old_r * np.sin(old_theta) * np.sin(old_phi)
    z = old_r * np.cos(old_theta)

    scale_factor = new_r / old_r
    x *= scale_factor
    y *= scale_factor
    z *= scale_factor

    new_theta, new_phi = convert_to_spherical(x, y, z)
    return new_theta, new_phi

old_theta = np.array([53.82653458, 58.20928332, 43.77046075, 38.21971664, 54.50928332, 53.82653458, 38.21971664, 41.77046075])
old_phi = np.array([346.4647812, 328.0076872, 319.4166341, 342.2518078, 213.9923128, 193.5352188, 197.7481922, 220.5833659])
old_r = 15.27276908
new_r = 19.77681679

new_theta, new_phi = adjust_distance(np.radians(old_theta), np.radians(old_phi), old_r, new_r)

print("New theta values:", np.degrees(new_theta).tolist())
print("New phi values:", np.degrees(new_phi).tolist())




