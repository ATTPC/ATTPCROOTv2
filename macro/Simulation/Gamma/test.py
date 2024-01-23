import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.optimize import curve_fit
from scipy.stats import lognorm
import numpy as np
import pandas as pd
data = pd.read_csv("efficiency_curve.csv")

# Extracting energy, efficiency, and error columns from the DataFrame
x_data = data["Energy"].values
efficiency = data["Efficiency"].values

# Define the log-normal function
def lognorm_func(x, s, loc, scale):
    return lognorm.pdf(x, s, loc, scale) * scale

# Plot the initial data
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.35)
p, = ax.plot(x_data, efficiency, 'o')
plt.title('Log-Normal Fit')

# Add sliders for fitting parameters
axcolor = 'lightgoldenrodyellow'
ax_s = plt.axes([0.1, 0.2, 0.65, 0.03], facecolor=axcolor)
ax_loc = plt.axes([0.1, 0.15, 0.65, 0.03], facecolor=axcolor)
ax_scale = plt.axes([0.1, 0.1, 0.65, 0.03], facecolor=axcolor)

s_slider = Slider(ax_s, 's', 0.1, 10.0, valinit=1)
loc_slider = Slider(ax_loc, 'loc', -10, 10.0, valinit=0)
scale_slider = Slider(ax_scale, 'scale', 0.1, 10.0, valinit=1)

# Update the plot with new sliders' values
def update(val):
    s = s_slider.val
    loc = loc_slider.val
    scale = scale_slider.val
    p.set_ydata(lognorm_func(x_data, s, loc, scale) * max(efficiency))
    fig.canvas.draw_idle()

# Call update function on slider value change
s_slider.on_changed(update)
loc_slider.on_changed(update)
scale_slider.on_changed(update)

plt.show()