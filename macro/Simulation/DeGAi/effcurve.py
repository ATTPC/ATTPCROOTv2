import os
import csv
import re
from pushbullet import Pushbullet
import numpy as np

#momentum_values = np.arange(0.1, 6.1, 0.1)
momentum_values= [0.1,1,2.0]
#momentum_value= [0.1,0.2,0.3,0.4,0.5,1,1.5,2,3,4,5]
output_lines = []
no_events = 50000
# Send a notification
pb = Pushbullet("o.ujiJPb5NJp4DaI6nQViea799x2UInEiP")
# Run gamma_sim.C and Simp_gamma_analysis.C for each momentum value
for momentum in momentum_values:
    # Compile and run gamma_sim.C with momentum as a command-line argument
    
    os.system(f"root -l -b -q 'gamma_sim.C({momentum},{no_events})'")

    # Compile and run Simp_gamma_analysis.C with momentum as a command-line argument
    output = os.popen(f"root -l -b -q 'Simp_gamma_analysis.C({momentum},{no_events})'").read()

    # Extract efficency and error from the terminal output
    efficency_match = re.search(r"Photopeak Efficency : ([\d.]+)%", output)
    error_match = re.search(r"Error:([\d.]+)", output)

    efficency = efficency_match.group(1) if efficency_match else "N/A"
    error = error_match.group(1) if error_match else "N/A"

    # Print efficency and error
    print(f"Momentum: {momentum}")
    print(f"Photopeak Efficency: {efficency}%")
    print(f"Error:{error}\n")

    # Store output line for CSV
    output_lines.append([momentum, efficency, error])

# Collect results in efficency_curve.csv
with open("efficency_curve.csv", "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Momentum", "Efficency", "Error"])
    writer.writerows(output_lines)


pb.push_note("Simulation Done", "The simulation and analysis are complete!")

# Open the CSV file in the default application
os.system("more efficency_curve.csv")
