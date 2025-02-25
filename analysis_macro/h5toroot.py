import h5py
import ROOT

def create_root_file_from_h5(h5_filename, root_filename):
    # Open the HDF5 file
    f = h5py.File(h5_filename, 'r')
    
    # Create a ROOT file
    root_file = ROOT.TFile(root_filename, "RECREATE")
    
    # Create a ROOT Tree to store the coordinates
    tree = ROOT.TTree("ClusterTree", "Tree with X, Y, Z coordinates")
    
    # Define variables for X, Y, Z coordinates as vectors
    x = ROOT.std.vector('float')()
    y = ROOT.std.vector('float')()
    z = ROOT.std.vector('float')()
    
    # Create branches for the tree
    tree.Branch("x", x)
    tree.Branch("y", y)
    tree.Branch("z", z)
    
    # Iterate over the clusters in each event
    cluster_group = f["cluster"]
    for event_name in cluster_group:
        print(f"Processing event: {event_name}")
        try:
            event_group = cluster_group[event_name]
        except KeyError as e:
            print(f"Error accessing event group: {e}")
            continue
        
        for cluster_name in event_group:
            print(f"  Processing cluster: {cluster_name}")
            try:
                cluster_group = event_group[cluster_name]
                cloud = cluster_group["cloud"]
            except KeyError as e:
                print(f"Error accessing cluster group or cloud dataset: {e}")
                continue
            
            # Clear the vectors to add new data
            x.clear()
            y.clear()
            z.clear()
            
            # Add points from cloud dataset to the tree
            for point in cloud:
                # Each point in the cloud has X, Y, Z coordinates in the first three columns
                x.push_back(point[0])  # X coordinate
                y.push_back(point[1])  # Y coordinate
                z.push_back(point[2])  # Z coordinate
            
            # Fill the tree with this cluster's data
            tree.Fill()
    
    # Write and close the ROOT file
    root_file.Write()
    root_file.Close()

# Usage example
create_root_file_from_h5("run_0116.h5", "run_0116_cluster.root")
