#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import json

with open('clusters.json') as json_file:
    data = json.load(json_file)

# Our 2-dimensional distribution will be over variables X and Y
N = 100
height_start, height_end = 0. , 0.3 
Intensity_start, Intensity_end = -10., 70.
X = np.linspace(Intensity_start, Intensity_end, N)
Y = np.linspace(height_start, height_end, N)
X, Y = np.meshgrid(X, Y)

# Mean vector and covariance matrix
a1 = (Intensity_end - Intensity_start)/(height_end - height_start)/10
a2 = (height_end - height_start)/(Intensity_end - Intensity_start)/10
Sigma = np.array([ [ a1  , 0.], [0., a2 ] ])

# Pack X and Y into a single 3-dimensional array
pos = np.empty(X.shape + (2,))
pos[:, :, 0] = X
pos[:, :, 1] = Y

#project v2 onto v1
def project_v1_on_v2(v1,v2):
    return np.linalg.norm(v1) * v1.dot(v2) * (v2/np.linalg.norm(v2))

def multivariate_gaussian(pos, mu, Sigma):
    """Return the multivariate Gaussian distribution on array pos.

    pos is an array constructed by packing the meshed arrays of variables
    x_1, x_2, x_3, ..., x_k into its _last_ dimension.

    """

    n = mu.shape[0]
    Sigma_det = np.linalg.det(Sigma)
    Sigma_inv = np.linalg.inv(Sigma)
    N = np.sqrt((2*np.pi)**n * Sigma_det)
    # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
    # way across all the input variables.
    fac = np.einsum('...k,kl,...l->...', pos-mu, Sigma_inv, pos-mu)

    return np.exp(-fac / 2) / N

min_i = float("inf")
max_i = -float("inf")
min_h = float("inf")
max_h = -float("inf")

n_clusters = len(data["clusters"])
print(n_clusters)
print(n_clusters/2)
print(math.ceil(n_clusters/2))
figure, axes = plt.subplots(2, 13)

blue_counter= 0
yellow_counter= 0
for n, cluster in enumerate(data["clusters"]):
    center = np.array(cluster["center_on_plane"])
    colour = 0 if center[1]>0 else 1
    normal = np.array(cluster["normal"])
    for x,y,z,i in cluster["points"]:
        min_i = min(i,min_i)
        max_i = max(i,max_i)
        
        v1 = np.array([x,y,z])
        v2 = v1-center

        h = np.linalg.norm(project_v1_on_v2(v1,normal))
        if h>0.27:
            continue
        min_h = min(h,min_h)
        max_h = max(h,max_h)

        mu = np.array([i,h])
        zz = multivariate_gaussian(pos, mu, Sigma)
    if "Z" in globals():
        if colour == 0:
            Z = zz
            axes[colour, blue_counter].imshow(zz)
            axes[colour, blue_counter].set_title("Colour: Blue")
            blue_counter+=1
        else:
            axes[colour, yellow_counter].imshow(zz)
            axes[colour, yellow_counter].set_title("Colour: Blue")
            Z += zz
            yellow_counter+=1
    else:
        if colour == 0:
            axes[colour, blue_counter].imshow(zz)
            axes[colour, blue_counter].set_title("Colour: Yellow")
            Z = -zz
            blue_counter+=1
        else:
            axes[colour, yellow_counter].imshow(zz)
            axes[colour, yellow_counter].set_title("Colour: Yellow")
            Z -= zz
            yellow_counter+=1
    

    if "Ztot" in globals():
        Ztot+=Z
    else:
        Ztot = Z

print("min max i:", min_i, max_i)
print("min max h:", min_h, max_h)

# Create a surface plot and projected filled contour plot under it.


fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(X, Y, Ztot, rstride=N/20, cstride=N/20, linewidth=1, antialiased=True, cmap=cm.viridis)

#cset = ax.contourf(X, Y, Z, zdir='z', offset=0, cmap=cm.viridis)

# Adjust the limits, ticks and view angle
ax.set_zlim(-50,50)
#ax.set_zticks(np.linspace(-100,100,5))
ax.view_init(30,235)
ax.set_xlabel('$Instensity$')
ax.set_ylabel('$Height$')
ax.set_zlabel('$Color$')

plt.show()