import os
import csv
import re

momentum_values = [i / 2 for i in range(1, 21)]
output_lines = []

# Run gamma_sim.C and Simp_gamma_analysis.C for each momentum value
for momentum in momentum_values:
    # Compile and run gamma_sim.C with momentum as a command-line argument
    os.system(f"root -l -b -q 'gamma_sim.C({momentum})'")

    # Compile and run Simp_gamma_analysis.C with momentum as a command-line argument
    output = os.popen(f"root -l -b -q 'Simp_gamma_analysis.C({momentum})'").read()

    # Extract efficiency and error from the terminal output
    efficiency_match = re.search(r"Photopeak Efficiency : ([\d.]+)%", output)
    error_match = re.search(r"Error:([\d.]+)", output)

    efficiency = efficiency_match.group(1) if efficiency_match else "N/A"
    error = error_match.group(1) if error_match else "N/A"

    # Print efficiency and error
    print(f"Momentum: {momentum}")
    print(f"Photopeak Efficiency: {efficiency}%")
    print(f"Error: {error}\n")

    # Store output line for CSV
    output_lines.append([momentum, efficiency, error])

# Collect results in efficiency_curve.csv
with open("efficiency_curve.csv", "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Momentum", "Efficiency", "Error"])
    writer.writerows(output_lines)

# Open the CSV file in the default application
os.system("gedit efficiency_curve.csv")

