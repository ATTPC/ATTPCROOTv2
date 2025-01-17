#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <TGraph.h>
#include <TCanvas.h>
#include <TLegend.h>

void read_txt_files() {
    // Manually specify the file names
    std::string file1 = "ex_ener_80_data.txt"; // Specify the name of the first file
    std::string file2 = "/home/david/Desktop/Azure2/AZURE2/azure_out_801.txt"; // Specify the name of the second file

    // Vectors to store the data from the first file
    std::vector<double> ecm1;
    std::vector<double> cross_section1;
    std::vector<double> cross_section1_errors;


    // Vectors to store the data from the second file
    std::vector<double> ecm2;
    std::vector<double> cross_section2;

    // Function to read data from a file and store it in vectors
    auto read_file = [](const std::string& filename, std::vector<double>& ecm, std::vector<double>& cross_section, std::vector<double>& cross_section_errors) {
        std::ifstream infile(filename);
        if (!infile.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }

        double col1, col2, col3, col4;
        while (infile >> col1 >> col2 >> col3 >> col4) {
            ecm.push_back(col1);
            cross_section.push_back(col3);
            cross_section_errors.push_back(col4); // Use the fourth column as Y error
            
        }

        infile.close();
    };

    // Function to read data from a file with 5 columns and store it in vectors
    auto read_file_5cols = [](const std::string& filename, std::vector<double>& ecm, std::vector<double>& cross_section, int col_to_read) {
        std::ifstream infile(filename);
        if (!infile.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return;
        }

        double col1, col2, col3, col4, col5;
        while (infile >> col1 >> col2 >> col3 >> col4 >> col5) {
            ecm.push_back(col1);
            if (col_to_read == 4) {
                cross_section.push_back(col4);
            }
        }

        infile.close();
    };

    // Read data from the first file (using the third column for cross_section)
    read_file(file1, ecm1, cross_section1, cross_section1_errors);

    // Read data from the second file (using the fourth column for cross_section)
    read_file_5cols(file2, ecm2, cross_section2, 4);

    for (auto& value : cross_section1) {
        value *= 0.001;
    }

    for (auto& value : cross_section1_errors) {
        value *= 0.001;
    }


    // Debugging: Print the data to verify correct reading
    std::cout << "Data from " << file1 << ":" << std::endl;
    for (size_t i = 0; i < ecm1.size(); ++i) {
        std::cout << "ecm1[" << i << "] = " << ecm1[i] << ", cross_section1[" << i << "] = " << cross_section1[i] << std::endl;
    }

    std::cout << "Data from " << file2 << ":" << std::endl;
    for (size_t i = 0; i < ecm2.size(); ++i) {
        std::cout << "ecm2[" << i << "] = " << ecm2[i] << ", cross_section2[" << i << "] = " << cross_section2[i] << std::endl;
    }

    // Create TGraph objects for the data
    TGraphErrors *graph1 = new TGraphErrors(ecm1.size(), ecm1.data(), cross_section1.data(), nullptr, cross_section1_errors.data());
    TGraph *graph2 = new TGraph(ecm2.size(), ecm2.data(), cross_section2.data());

    // Set graph styles
    graph1->SetMarkerStyle(20);
    graph1->SetMarkerColor(kBlue);
    graph1->SetLineColor(kBlue);
    graph1->SetTitle("Cross Section vs ECM");

    graph2->SetMarkerStyle(21);
    graph2->SetMarkerColor(kRed);
    graph2->SetLineColor(kRed);

    // Create a canvas and draw the graphs
    TCanvas *c1 = new TCanvas("c1", "Cross Section vs ECM", 800, 600);
    graph1->Draw("AP");
    graph2->Draw("P SAME");

    // Add a legend
    TLegend *legend = new TLegend(0.7, 0.7, 0.9, 0.9);
    legend->AddEntry(graph1, "Data", "lp");
    legend->AddEntry(graph2, "Azure", "lp");
    legend->Draw();

    // Save the canvas to a file
    c1->SaveAs("cross_section_vs_ecm.png");
}

// Example usage
int compare_azure() {
    read_txt_files();
    return 0;
}