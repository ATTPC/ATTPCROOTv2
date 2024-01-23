import os
import subprocess

def modify_script(isotope):
    with open("Gamma_sim.C", "r") as file:
        lines = file.readlines()

    with open("Gamma_sim.C", "w") as file:
        for line in lines:
            if line.strip().startswith("TString isotopeName ="):
                file.write(f'    TString isotopeName = "{isotope}";\n')
            else:
                file.write(line)

def run_simulation(isotope):
    modify_script(isotope)
    subprocess.run(["root", "-l","-b", "-q", "Gamma_sim.C"])

def main():
    isotopes = ["Co60", "Cs137", "I131", "Te99", "Na22", "Am241", "Th208", "U238"]
    
    # Backup the original script
    subprocess.run(["cp", "Gamma_sim.C", "Gamma_sim_backup.C"])

    for isotope in isotopes:
        run_simulation(isotope)

    # Restore the original script
    subprocess.run(["mv", "Gamma_sim_backup.C", "Gamma_sim.C"])

if __name__ == "__main__":
    main()
