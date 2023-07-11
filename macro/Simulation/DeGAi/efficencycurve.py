import numpy as np
import matplotlib.pyplot as plt

# Input values
momentum = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0]
efficiency = [9.73095, 8.2271, 8.56991, 8.62356, 8.72387, 8.58714, 8.94611, 8.58199, 9.15402, 9.50709, 8.72062,
              7.90622, 8.05356, 8.14994, 8.44895, 8.36013, 7.08746, 8.00551, 7.22587, 7.25059]
error = [0.309114, 0.262671, 0.255392, 0.24988, 0.244126, 0.232937, 0.228042, 0.215972, 0.213811, 0.210956,
         0.196279, 0.181286, 0.17783, 0.172895, 0.170974, 0.168215, 0.149783, 0.15649, 0.147497, 0.142443]

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
plt.text(1.5, 10, f'Curve Formula: {formula}', fontsize=12, bbox={'facecolor': 'white', 'alpha': 0.5})

# Displaying the plot
plt.show()
