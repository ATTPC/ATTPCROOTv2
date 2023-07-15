import numpy as np
import matplotlib.pyplot as plt
import csv

# Reading data from file
momentum = []
efficiency = []
error = []

with open("efficiency_curve.csv", "r") as file:
    reader = csv.reader(file)
    next(reader)  # Skip the header row
    for row in reader:
        momentum.append(float(row[0]))
        efficiency.append(float(row[1]))
        error.append(float(row[2]))

# Polynomial regression
degree = 3  # Adjust the degree of the polynomial fit as desired
coefficients = np.polyfit(momentum, efficiency, degree)
polynomial = np.poly1d(coefficients)
formula = polynomial.__str__()

# Plotting the data and the best-fit curve
plt.figure(figsize=(10, 6))
plt.errorbar(momentum, efficiency, yerr=error, fmt='o', color='b', ecolor='g', capsize=5, label='Data')
plt.plot(momentum, polynomial(momentum), c='r', label='Best-fit Curve')
plt.xlabel('Momentum')
plt.ylabel('Efficiency')
plt.title('Degai Efficiency Curve')
plt.legend()
plt.grid(True)

# Prompting the user to input momentum values to add to the plot
num_points = int(input("Number of predicted points: "))
user_momentum = []
if num_points > 0:
    for i in range(num_points):
        momentum_value = float(input(f"Enter momentum value {i+1}: "))
        user_momentum.append(momentum_value)

    user_efficiency = polynomial(user_momentum)
    plt.plot(user_momentum, user_efficiency, 'ro', label='User Inputted Efficiency')

# Displaying the formula
plt.text(0.5, 7, f'Curve Formula: {formula}', fontsize=12, bbox={'facecolor': 'white', 'alpha': 0.5})

# Displaying the plot
plt.show()
