# %%
import numpy as np
import h5py
import math
#from sklearn.decomposition import PCA
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import os, sys
#from BaselineRemoval import BaselineRemoval
import scipy
import itertools
import random

# assign directory
h5_directory = "./data/"

reset = True
if reset: # remove output files
    try:
        os.remove(h5_directory + "numerical-full.csv")
        os.remove(h5_directory + "numerical-summary.csv")
    except:
        pass

# %%
def Analyze_H5(h5_dir, file, threshold, ADD_NOISE=False):
    def pca_func(data, n_components):
        
        # original method
        #pca = PCA(n_components=n_components)
        #pca.fit(data)
        #out = pca.transform(data)
        
        # fishtank-compatible method
        # Subtract the mean from the data to center it
        if(data == 0 ):
            mean = 0
        else:
            mean = np.mean(data, axis=0)
        data_centered = data - mean
        
        # Compute the covariance matrix
        cov = np.cov(data_centered, rowvar=False)

        # Compute the eigenvectors and eigenvalues of the covariance matrix
        eigvals, eigvecs = np.linalg.eig(cov)

        # Transform the data into the new coordinate system defined by the eigenvectors
        out = np.dot(data_centered, eigvecs)

        return out
    
    def dbscan(X, eps, min_samples):
        # Initialize variables
        labels = np.zeros(X.shape[0])
        cluster = 0

        # Compute distances between points
        dists = np.sqrt(((X[:, np.newaxis] - X) ** 2).sum(axis=2))

        # Iterate over each point
        for i in range(X.shape[0]):
            # If point already visited, continue
            if labels[i] != 0:
                continue

            # Find neighboring points
            neighbors = np.where(dists[i] <= eps)[0]

            # If not enough neighboring points, label as noise
            if len(neighbors) < min_samples:
                labels[i] = -1
                continue

            # Expand cluster
            cluster += 1
            labels[i] = cluster

            while len(neighbors) > 0:
                j = neighbors[0]
                if labels[j] == -1:
                    labels[j] = cluster
                elif labels[j] == 0:
                    labels[j] = cluster
                    new_neighbors = np.where(dists[j] <= eps)[0]
                    if len(new_neighbors) >= min_samples:
                        neighbors = np.concatenate((neighbors, new_neighbors))
                neighbors = neighbors[1:]

        return labels
    
    def remove_outliers(xset, yset, zset, eset, threshold):
        #Uses DBSCAN to find and remove outliers in 3D data
        # NEEDS ALTERNATE METHOD FOR FISHTANK COMPATIBILITY
        data = np.array([xset.T, yset.T, zset.T]).T

        # STANDARD METHOD
        #DBSCAN_cluster = DBSCAN(eps=7, min_samples=10).fit(data)
        #out_of_cluster_index = np.where(DBSCAN_cluster.labels_==-1)

        # FISHTANK METHOD
        labels = dbscan(data, eps=7, min_samples=10)
        out_of_cluster_index = np.where(labels==-1)

        del data
        rev = out_of_cluster_index[0][::-1]

        for i in rev:
            xset = np.delete(xset, i)
            yset = np.delete(yset, i)
            zset = np.delete(zset, i)
            eset = np.delete(eset, i)
        if len(xset) <= threshold:
            veto = True
        else:
            veto = False

        # testing
        veto = False
        
        return xset, yset, zset, eset, veto
    
    def track_len(xset, yset, zset):
        """
        Uses PCA to find the length of a track
        """
        veto_on_length = False
    
        # Form data matrix
        data = np.concatenate((xset[:, np.newaxis], 
                               yset[:, np.newaxis], 
                               zset[:, np.newaxis]), 
                               axis=1)
    
        # Use PCA to find track length
        principalComponents = pca_func(data, 3)
    
        principalDf = pd.DataFrame(data = principalComponents
                 , columns = ['principal component 1', 'principal component 2', 'principal component 3'])
        
        track_len = 2.35*principalDf.std()[0]
        track_width = 2.35*principalDf.std()[1]
        track_depth = 2.35*principalDf.std()[2]
        #if track_len > 70:
        #    veto_on_length = True
        
        return track_len, veto_on_length, track_width, track_depth
    
    def main(h5file, threshold, ADD_NOISE):
        """
        This functions does the following: 
        - Converts h5 files into ndarrays. 
        - Removes outliers.
        - Calls PCA to return track length.
        - Sums mesh signal to return energy.
        """
        # Converts h5 files into ndarrays, and output each event dataset as a separte list
        num_events = int(len(list(h5file.keys()))) 
        
        good_events = []
        
        length_list = []
        width_list = []
        depth_list = []
        
        tot_energy = []
        tracemax_list = []
        peakw_list = []
        
        maxe_list = []
        stde_list = []
        padnum_list = []
        
        skipped_events = 0
        veto_events = 0
        
        #pbar = tqdm(total=num_events+1)
        for i in range(0, num_events):
            #print(f"Event {i}")
            str_event = f"Event_[{i}]"
            
            # Apply pad threshold
            event = h5file[str_event]['HitArray']
            if len(event) <= threshold:
                skipped_events += 1
                print('skipped event', i ,'due to pad threshold')
                continue
                
            # Make copy of datasets
            dset_0_copyx = event['x']
            dset_0_copyy = event['y'] 
            dset_0_copyz = event['z'] - min(event['z'])
            dset_0_copye = event['A']
            
            # Apply veto condition
            R = 36                           # Radius of the pad plane
            r = np.sqrt(dset_0_copyx**2 + dset_0_copyy**2)
            statements = np.greater(r, R)    # Check if any point lies outside of R
        
            #if np.any(statements) == True:
            #    veto_events += 1
            #    print('skipped event', i ,'due to veto condition')
            #    continue
            
            
            # Call remove_outliers to get dataset w/ outliers removed
            dset_0_copyx, dset_0_copyy, dset_0_copyz, dset_0_copye, veto = remove_outliers(dset_0_copyx, dset_0_copyy, dset_0_copyz, dset_0_copye, threshold)
            veto = False
            if veto == True:
                skipped_events += 1
                print('skipped event', i ,'due to outlier removal')
                continue

            
            # Call track_len() to create lists of all track lengths
            length, veto_on_length, width, depth = track_len(dset_0_copyx, dset_0_copyy, dset_0_copyz)
            if veto_on_length == True:
                veto_events += 1
                print('skipped event', i ,'due to track length')
                continue

            
            #str_trace = f"Trace_[{i}]"
            trace = h5file[str_event]['Trace']
            max_val = np.argmax(trace)
            low_bound = max_val - 75
            if low_bound < 0:
                low_bound = 5
            upper_bound = max_val + 75
            if upper_bound > 511:
                upper_bound = 506
            trace = trace[low_bound:upper_bound]

            # STANDARD METHOD
            #baseObj=BaselineRemoval(trace)
            #trace=baseObj.IModPoly(polynomial_degree)

            # FISHTANK METHOD
            # determine the width of the peak in the trace and the location of the peak
            peakloc = np.argmax(trace)
            
            peakwidth1 = 0
            peakwidth2 = 0

            k = peakloc
            while trace[k] > np.min(trace) + np.std(trace):
                peakwidth1 += 1
                k += 1
            k = peakloc
            while trace[k] > np.min(trace) + np.std(trace):
                peakwidth2 += 1
                k -= 1

            # calculate the average of the trace outside of the peakwidth window on either side of the peak
            baseline = np.mean(np.concatenate((trace[:peakloc-peakwidth2], trace[peakloc+peakwidth1:])))
            # subtract the baseline from the trace
            trace = trace - baseline

            # ADD NOISE TO TRACE
            # noise of frequency ~0.25 with random phase
            if ADD_NOISE:
                noise_peak = np.sin(np.linspace(0, 2*np.pi*len(trace)*random.gauss(2.49815e-1, -8.27021e-3), len(trace)) + random.random()*2*np.pi)

                noise_background = np.zeros_like(trace)
                for freq in np.linspace(0.1, 0.5):
                    noise_background += np.sin(np.linspace(0, 2*np.pi*len(trace)*freq, len(trace)) + random.random()*2*np.pi) * random.gauss(1, 0.3)
                noise_background = noise_background - np.mean(noise_background)
                noise_background = noise_background / np.std(noise_background)


                noise = 13000*noise_peak + 2000 * noise_background

                # normalize noise to std of 1 and average of 0, then multiply by amplitude of real data (402.57 +- 106.40 std)
                noise = (noise - np.mean(noise))/np.std(noise) * random.gauss(402.57, 106.40)
                # add noise to trace
                trace = trace + noise
            
            # VETO ON TRACE SUM
            #if np.sum(trace) > 800000:
            #    veto_events += 1
            #    pbar.update(n=1)
            #    continue

            length_list.append(length)
            width_list.append(width)
            depth_list.append(depth)
            
            tot_energy.append(np.sum(trace))
            tracemax_list.append(np.max(trace))
            peakw_list.append(peakwidth1 + peakwidth2)
            
            maxe_list.append(np.max(dset_0_copye))
            stde_list.append(np.std(dset_0_copye))
            padnum_list.append(len(dset_0_copyx))    

            # Track event number of good events
            good_events.append(i)  
            #pbar.update(n=1)
            
        # package lists into a dictionary
        output_dict = {
            "length": length_list,
            "width": width_list,
            "depth": depth_list,
            "trace_sum": tot_energy,
            "trace_max": tracemax_list,
            "peak_width": peakw_list,
            "max_e": maxe_list,
            "std_e": stde_list,
            "padnum": padnum_list}
        
        return output_dict
    
    h5f = h5py.File(h5_dir + file, 'r')
    
    output_dict = main(h5file=h5f, threshold=5, ADD_NOISE=ADD_NOISE)
    
    results_df = pd.DataFrame(columns=['file', 'ptype', 'event#',
                                       'length', 'width', 'depth',
                                       'trace_sum', 'trace_max', 'peak_width',
                                       'max_e', 'std_e', 'padnum'])
    
    file_name = file.split('.h5')[0]
    ptype = 'testing' #file_name.split('-')[-1]
    
    for i in range(len(output_dict['length'])):
        results_df.loc[i] = [file_name, ptype, i,
                             output_dict['length'][i], output_dict['width'][i], output_dict['depth'][i],
                             output_dict['trace_sum'][i], output_dict['trace_max'][i], output_dict['peak_width'][i],
                             output_dict['max_e'][i], output_dict['std_e'][i], output_dict['padnum'][i]]
        
    return results_df
# %%
# open or create output file
try:
    output_df = pd.read_csv(h5_directory + 'numerical-full.csv')
except FileNotFoundError:
    output_df = pd.DataFrame(columns=['file', 'ptype', 'event#',
                                           'length', 'width', 'depth',
                                           'trace_sum', 'trace_max', 'peak_width',
                                           'max_e', 'std_e', 'padnum'])
# loop through all files in directory
for file in os.listdir(h5_directory):
    if file.endswith(".h5") and file not in output_df['file'].values:
        print(file)
        output_df = output_df.append(Analyze_H5(h5_directory, file, ADD_NOISE=False, threshold=0))

output_df.to_csv(h5_directory + 'numerical-full.csv', index=False)

# %%
# condense output file to summary file
output_df['padnum'] = output_df['padnum'].astype(float)
summary_df = output_df.groupby(['file']).mean().reset_index()
# add standard deviation columns
summary_df['length_std'] = output_df.groupby(['file'])['length'].std().reset_index()['length']
summary_df['width_std'] = output_df.groupby(['file'])['width'].std().reset_index()['width']
summary_df['depth_std'] = output_df.groupby(['file'])['depth'].std().reset_index()['depth']
summary_df['trace_sum_std'] = output_df.groupby(['file'])['trace_sum'].std().reset_index()['trace_sum']
summary_df['trace_max_std'] = output_df.groupby(['file'])['trace_max'].std().reset_index()['trace_max']
summary_df['peak_width_std'] = output_df.groupby(['file'])['peak_width'].std().reset_index()['peak_width']
summary_df['max_e_std'] = output_df.groupby(['file'])['max_e'].std().reset_index()['max_e']
summary_df['std_e_std'] = output_df.groupby(['file'])['std_e'].std().reset_index()['std_e']
summary_df['padnum_std'] = output_df.groupby(['file'])['padnum'].std().reset_index()['padnum']

summary_df.to_csv(h5_directory + 'numerical-summary.csv', index=False)