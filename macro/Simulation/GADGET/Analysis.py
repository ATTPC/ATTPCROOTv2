import h5py
import numpy as np
import matplotlib.pyplot as plt
def analyze_h5_files():
    h5_file1 = "./data/CD_digi.h5"
    h5_file2 = "./data/diff_digi.h5"
    with h5py.File(h5_file1, 'r') as f1, h5py.File(h5_file2, 'r') as f2:
        # Print the keys of the first H5 file
        print("find x:")
    
        print(f1['Event_[1]']['HitArray']['x'])
    
        # Access the variables from the first H5 file
        x1 = f1['x'][:]
        y1 = f1['y'][:]
        z1 = f1['z'][:]
        t1 = f1['t'][:]
        A1 = f1['A'][:]
        trackID1 = f1['trackID'][:]
        pointIDMC1 = f1['pointIDMC'][:]
        energyMC1 = f1['energyMC'][:]
        elossMC1 = f1['elossMC'][:]
        angleMC1 = f1['angleMC'][:]
        AMC1 = f1['AMC'][:]
        ZMC1 = f1['ZMC'][:]

        # Access the variables from the second H5 file
        x2 = f2['x'][:]
        y2 = f2['y'][:]
        z2 = f2['z'][:]
        t2 = f2['t'][:]
        A2 = f2['A'][:]
        trackID2 = f2['trackID'][:]
        pointIDMC2 = f2['pointIDMC'][:]
        energyMC2 = f2['energyMC'][:]
        elossMC2 = f2['elossMC'][:]
        angleMC2 = f2['angleMC'][:]
        AMC2 = f2['AMC'][:]
        ZMC2 = f2['ZMC'][:]

        # Perform statistics and analysis on the first file
        num_tracks1 = np.unique(trackID1).size
        total_energy1 = np.sum(energyMC1)
        avg_eloss1 = np.mean(elossMC1)
        max_angle1 = np.max(angleMC1)
        min_z1 = np.min(z1)
        max_z1 = np.max(z1)
        
        # Perform statistics and analysis on the second file
        num_tracks2 = np.unique(trackID2).size
        total_energy2 = np.sum(energyMC2)
        avg_eloss2 = np.mean(elossMC2)
        max_angle2 = np.max(angleMC2)
        min_z2 = np.min(z2)
        max_z2 = np.max(z2)
        
        # Print the statistics for the first file
        print("Statistics for File 1:")
        print("Number of tracks:", num_tracks1)
        print("Total energy:", total_energy1)
        print("Average energy loss:", avg_eloss1)
        print("Maximum angle:", max_angle1)
        print("Minimum z-coordinate:", min_z1)
        print("Maximum z-coordinate:", max_z1)
        print()

        # Print the statistics for the second file
        print("Statistics for File 2:")
        print("Number of tracks:", num_tracks2)
        print("Total energy:", total_energy2)
        print("Average energy loss:", avg_eloss2)
        print("Maximum angle:", max_angle2)
        print("Minimum z-coordinate:", min_z2)
        print("Maximum z-coordinate:", max_z2)
        print()

        # Additional comparison or analysis can be performed here

 # Generate histogram using values in A
        plt.hist(A1, bins=10, alpha=0.5, label='File 1')
        plt.hist(A2, bins=10, alpha=0.5, label='File 2')
        plt.xlabel('A values')
        plt.ylabel('Frequency')
        plt.title('Histogram of A values')
        plt.legend()
        plt.show()


# Call the analyze_h5_file function
analyze_h5_files()