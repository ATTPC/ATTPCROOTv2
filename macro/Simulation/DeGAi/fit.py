import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Reading data from a file into a pandas DataFrame (dummy data for testing purposes)
data = pd.DataFrame({'Energy': np.linspace(1,10,10), 'Efficiency': np.linspace(1,10,10), 'Error': np.linspace(0.1,1,10)})

# Extracting energy, efficiency, and error columns from the DataFrame
energy = data["Energy"].values
efficiency = data["Efficiency"].values
error = data["Error"].values

def custom_curve_function(energy, C1, C2, C3, C4, C5):
    return np.sum([C_i * np.log(energy)**i for i, C_i in enumerate([C1, C2, C3, C4, C5])])

class App:
    def __init__(self, master):
        self.master = master
        master.title("Curve Fitting GUI")

        # Initial guesses
        self.guesses = {'C1': tk.DoubleVar(value=10.0),
                        'C2': tk.DoubleVar(value=-1.0),
                        'C3': tk.DoubleVar(value=0.1),
                        'C4': tk.DoubleVar(value=0.01),
                        'C5': tk.DoubleVar(value=0.001)}

        # Entry widgets for initial guesses
        row = 0
        for param, var in self.guesses.items():
            lbl = ttk.Label(master, text=f"{param}:")
            lbl.grid(column=0, row=row, sticky="w")
            ent = ttk.Entry(master, textvariable=var)
            ent.grid(column=1, row=row)
            row += 1

        # Button to update plot
        self.plot_button = ttk.Button(master, text="Update Plot", command=self.update_plot)
        self.plot_button.grid(column=0, row=row, columnspan=2)

        # Create initial plot
        self.fig, self.ax = plt.subplots(figsize=(6,4))
        self.canvas = FigureCanvasTkAgg(self.fig, master)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(column=2, row=0, rowspan=row+1)

        # Initial plot update
        self.update_plot()

    def update_plot(self):
        # Clear previous plot
        self.ax.clear()

        # Extract guesses
        guesses = [var.get() for var in self.guesses.values()]

        # Perform the curve fit
        fit_params, _ = curve_fit(custom_curve_function, energy, efficiency, p0=guesses)

        # Plotting the data
        self.ax.errorbar(energy, efficiency, yerr=error, fmt='+', color='b', ecolor='g', capsize=5, label='Data')
        energy_range = np.linspace(min(energy), max(energy), 100)
        fitted_efficiency = [custom_curve_function(e, *fit_params) for e in energy_range]
        self.ax.plot(energy_range, fitted_efficiency, c='r', label='Best-fit Curve')
        self.ax.set_xlabel('Energy')
        self.ax.set_ylabel('Efficiency')
        self.ax.set_title('Custom Efficiency Curve')
        self.ax.legend()
        self.ax.grid(True)

        # Redraw the updated plot
        self.canvas.draw()

root = tk.Tk()
app = App(root)
root.mainloop()
