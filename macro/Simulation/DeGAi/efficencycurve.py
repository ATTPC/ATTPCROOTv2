import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import curve_fit
from scipy.stats import lognorm

# Reading data from a file into a pandas DataFrame
data = pd.read_csv("efficiency_curve.csv")

# Extracting energy, efficiency, and error columns from the DataFrame
energy = data["Energy"].values
efficiency = data["Efficiency"].values
error = data["Error"].values

# Log-normal function to fit
def func(x, s, loc, scale):
    return lognorm.pdf(x, s, loc=loc, scale=scale) * scale  # The scale factor is used to adjust the y-values to the data range.

# Curve fitting
popt, pcov = curve_fit(func, energy, efficiency)

# Plotting the data and the best-fit curve
plt.figure(figsize=(10, 6))
plt.errorbar(energy, efficiency, yerr=error, fmt='+', color='b', ecolor='g', capsize=5, label='Data')
plt.plot(energy, func(energy, *popt), 'r-', label='Fit: s=%5.3f, loc=%5.3f, scale=%5.3f' % tuple(popt))

plt.xlabel('Energy')
plt.ylabel('Efficiency')
plt.title('DeGai Efficiency Curve')
plt.legend()
plt.grid(True)

# Displaying the fitted formula
print(f"Fitted Parameters:\n s: {popt[0]:.2f}\n loc: {popt[1]:.2f}\n scale: {popt[2]:.2f}")
#plt.ylim(-0.1, 1.1)  # Adjust the y-axis limits as needed
plt.show()
