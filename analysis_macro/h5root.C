#include <vector>
#include <string>
#include <H5Cpp.h>
#include <TFile.h>
#include <TTree.h>
#include <iostream>
#include <regex>

using namespace std;
using namespace H5;

void processH5File(const string& h5_file, const string& root_file) {
    // Open the HDF5 file
    H5File file(h5_file, H5F_ACC_RDONLY);

    // Create a ROOT file
    TFile* rootFile = new TFile(root_file.c_str(), "RECREATE");

    // Create a ROOT Tree to store the coordinates
    TTree* tree = new TTree("ClusterTree", "Tree with Event ID, Cluster ID, X, Y, Z coordinates");

    // Define variables for Event ID, Cluster ID, X, Y, Z coordinates, and cluster size
    int event_id, cluster_id;
    vector<float> x, y, z;
    int cluster_size;

    // Create branches for the tree
    tree->Branch("event_id", &event_id, "event_id/I");
    tree->Branch("cluster_id", &cluster_id, "cluster_id/I");
    tree->Branch("x", &x);
    tree->Branch("y", &y);
    tree->Branch("z", &z);
    tree->Branch("cluster_size", &cluster_size, "cluster_size/I");

    // Navigate through the 'cluster' group in HDF5 file
    Group clusterGroup = file.openGroup("/cluster");

    cout << "Number of events: " << clusterGroup.getNumObjs() << endl;

    // Iterate over the events (event_i)
    int consecutive_errors = 0;
    bool previous_error = false;
    int eventIdx = 0;

    while (consecutive_errors < 5) {
        // Set the event ID
        if (!previous_error)
            consecutive_errors = 0;

        previous_error = false;
        event_id = eventIdx;

        // Construct the event name
        string eventName = "event_" + to_string(eventIdx);
        cout << "Processing event: " << eventName << " (ID: " << event_id << ")" << endl;

        Group eventGroup;

        try {
            eventGroup = clusterGroup.openGroup(eventName.c_str());
            cout << "Opened event group: " << eventName << endl;
        } catch (const GroupIException& e) {
            cerr << "Warning: Event group " << eventName << " does not exist. Skipping." << endl;
            consecutive_errors++;
            previous_error = true;
            eventIdx++;
            continue;
        }

        // Check if the event group contains any clusters
        if (eventGroup.getNumObjs() == 0) {
            cerr << "Warning: Event group " << eventName << " contains no clusters. Skipping." << endl;
            eventIdx++;
            continue;
        }

        // Iterate over the clusters in the event (cluster_i)
        for (size_t clusterIdx = 0; clusterIdx < eventGroup.getNumObjs(); ++clusterIdx) {
            try {
                // Set the cluster ID
                cluster_id = clusterIdx;

                // Get the size of the cluster name
                ssize_t clusterNameSize = eventGroup.getObjnameByIdx(clusterIdx, nullptr, 0);
                // Allocate buffer for the cluster name
                vector<char> clusterNameBuffer(clusterNameSize + 1);
                // Get the cluster name
                eventGroup.getObjnameByIdx(clusterIdx, clusterNameBuffer.data(), clusterNameSize + 1);
                string clusterName(clusterNameBuffer.data());

                cout << "  Processing cluster: " << clusterName << " (ID: " << cluster_id << ")" << endl;

                Group clusterGroup = eventGroup.openGroup(clusterName.c_str());
                cout << "  Opened cluster group: " << clusterName << endl;

                // Open the 'cloud' dataset for the current cluster
                DataSet cloudDataset = clusterGroup.openDataSet("cloud");
                cout << "  Opened cloud dataset for cluster: " << clusterName << endl;

                // Get the dimensions of the dataset (assumes it has 5 columns: X, Y, Z, and two more)
                DataSpace dataspace = cloudDataset.getSpace();
                hsize_t dims[2];
                int ndims = dataspace.getSimpleExtentDims(dims, NULL);

                // Check if the dataset dimensions are read correctly
                if (ndims != 2 || dims[1] != 5) {
                    cerr << "Unexpected dataset dimensions for event: " << event_id << ", cluster: " << cluster_id << endl;
                    continue;
                }

                cluster_size = dims[0];  // Number of points in the cloud

                // Debugging statement to check the number of points
                cout << "    Number of points in cluster: " << cluster_size << endl;

                // Resize the vectors to hold the data
                x.resize(cluster_size);
                y.resize(cluster_size);
                z.resize(cluster_size);

                // Read the data into the vectors
                vector<float> data(cluster_size * 5);  // Assuming 5 columns
                cloudDataset.read(data.data(), PredType::NATIVE_FLOAT);

                // Copy the data into the vectors
                for (hsize_t i = 0; i < cluster_size; ++i) {
                    x[i] = data[i * 5];
                    y[i] = data[i * 5 + 1];
                    z[i] = data[i * 5 + 2];
                }

                // Fill the tree with the data
                tree->Fill();
            } catch (const GroupIException& e) {
                cerr << "Warning: Cluster group " << clusterIdx << " in event " << eventName << " does not exist or is invalid. Skipping." << endl;
                continue;
            } catch (const DataSetIException& e) {
                cerr << "Warning: Dataset in cluster " << clusterIdx << " in event " << eventName << " does not exist or is invalid. Skipping." << endl;
                continue;
            } catch (const DataSpaceIException& e) {
                cerr << "Warning: Dataspace in cluster " << clusterIdx << " in event " << eventName << " is invalid. Skipping." << endl;
                continue;
            }
        }

        eventIdx++;
    }

    // Write the tree to the ROOT file and close the files
    tree->Write();
    rootFile->Close();
    file.close();
}

void h5root() {
    // List of HDF5 files to process
    vector<string> h5_files = {
      /*  "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0116.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0117.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0118.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0119.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0120.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0134.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0135.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0136.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0137.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0138.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0139.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0140.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0141.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0143.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0144.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0145.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0146.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0147.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0148.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0149.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0150.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0153.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0156.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0157.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0158.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0159.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0160.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0161.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0162.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0163.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0164.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0165.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0166.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0167.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0168.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0169.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0170.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0171.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0172.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0173.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0174.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/highener/Cluster/run_0175.h5"*/
        // Add more file paths as needed

       /* "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0062.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0063.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0064.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0065.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0066.h5",
      //  "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0067.h5",
      //  "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0070.h5",
      //  "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0071.h5",
      //  "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0072.h5",
      //  "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0073.h5",
      //  "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0074.h5",
      //  "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0075.h5",
      //  "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0076.h5",
      //  "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0077.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0078.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0079.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0080.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0081.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0082.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0083.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0084.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0085.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0086.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0087.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0089.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0090.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0091.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0092.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0093.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0094.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0095.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0096.h5",
        //"/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0097.h5",*/
       // "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0098.h5",
        //"/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0099.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0100.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0101.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0102.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0103.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0104.h5",
        "/home/david/PhD/PhD-14-02/attpcroot/spyral_fit/lowener/Cluster/run_0105.h5"
    };

    // Process each HDF5 file and create a separate ROOT file for each
    for (size_t i = 0; i < h5_files.size(); ++i) {
        const string& h5_file = h5_files[i];
        // Generate the corresponding ROOT file name
        string root_file = h5_file.substr(0, h5_file.find_last_of(".")) + "_cluster.root";
        std::cout << "Processing file " << (i + 1) << " of " << h5_files.size() << ": " << h5_file << std::endl;
        processH5File(h5_file, root_file);
    }
}

int main() {
    h5root();
    return 0;
}