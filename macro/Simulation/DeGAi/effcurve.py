import os
import csv
import re
from pushbullet import Pushbullet
import numpy as np

energy_values = [2]
no_events = [750000]

pb = Pushbullet("o.ujiJPb5NJp4DaI6nQViea799x2UInEiP")

output_lines = []

for energy in energy_values:
    for no_event in no_events:
        os.system(f"root -l -b -q 'gamma_sim.C({energy},{no_event})'")
        
        output = os.popen(f"root -l -b -q 'Simp_gamma_analysis.C({energy},{no_event})'").read()

        # Extract all relevant values from the terminal output
        num_ev_match = re.search(r"Total number of events : (\d+)", output)
        PhotopeakCount_match = re.search(r"Number of events in photopeak : (\d+)", output)
        efficiency_match = re.search(r"Photopeak Efficency : ([\d.]+)%", output)
        error_match = re.search(r"Error: ([\d.]+)", output)

        num_ev = num_ev_match.group(1) if num_ev_match else "N/A"
        PhotopeakCount = PhotopeakCount_match.group(1) if PhotopeakCount_match else "N/A"
        efficiency = efficiency_match.group(1) if efficiency_match else "N/A"
        error = error_match.group(1) if error_match else "N/A"

        # Print values
        print(f"Energy: {energy}")
        print(f"Total number of events: {num_ev}")
        print(f"Number of events in photopeak: {PhotopeakCount}")
        print(f"Photopeak Efficiency: {efficiency}%")
        print(f"Error: {error}\n")

        # Store output line for CSV
        output_lines.append([no_event, energy, PhotopeakCount, efficiency, error])

        # Open the CSV file in append mode and write the output line
        with open("efficiency_curve.csv", "a", newline='') as f:
            writer = csv.writer(f)
            writer.writerow([no_event, energy, num_ev, PhotopeakCount, efficiency, error])

pb.push_note("Simulation Done", "The simulation and analysis are complete!")

os.system("more efficiency_curve.csv")
