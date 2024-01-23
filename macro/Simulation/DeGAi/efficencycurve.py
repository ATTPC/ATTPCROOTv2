import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Reading data from file into a pandas DataFrame
data = pd.read_csv("efficiency_curve.csv")

# Extracting momentum, efficiency, and error columns from the DataFrame
momentum = data["Momentum"].values
efficiency = data["Efficiency"].values
error = data["Error"].values

# Rest of the code remains the same...

# Polynomial regression
degree = 2  # Adjust the degree of the polynomial fit as desired
coefficients = np.polyfit(momentum, efficiency, degree)
polynomial = np.poly1d(coefficients)
formula = polynomial.__str__()

# Plotting the data and the best-fit curve
plt.figure(figsize=(10, 6))
plt.errorbar(momentum, efficiency, yerr=error, fmt='o', color='b', ecolor='g', capsize=5, label='Data')
plt.plot(momentum, polynomial(momentum), c='r', label='Best-fit Curve')
plt.xlabel('Energy (MeV)')
plt.ylabel('Efficiency (%)')
plt.title('Degai Efficiency Curve')
plt.legend()
plt.grid(True)

# Prompting the user to input momentum values to add to the plot
num_points = int(input("Number of predicted points: "))
user_momentum = []
if num_points > 0:
    for i in range(num_points):
        momentum_value = float(input(f"Enter Energy value {i+1}: "))
        user_momentum.append(momentum_value)

    user_efficiency = polynomial(user_momentum)
    plt.plot(user_momentum, user_efficiency, 'ro', label='User Inputted Efficiency')

# Displaying the formula
plt.text(0.5, 5, f'Curve Formula: {formula}', fontsize=12, bbox={'facecolor': 'white', 'alpha': 0.5})

# Displaying the plot
plt.show()

