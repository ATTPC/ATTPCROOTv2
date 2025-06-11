#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include "TH1F.h"
#include "TCanvas.h"

void SiArray_acc() {
    std::ifstream file("SiArray_acc.csv");
    if (!file.is_open()) {
        std::cerr << "Could not open SiArray_acc.csv" << std::endl;
        return;
    }

    // Skip the first three lines
    std::string line;
    std::getline(file, line);
    std::getline(file, line);
    std::getline(file, line);

    std::vector<double> y3, y6, y9, y12, y15, y18;
    int line_counter = 0;
    while (std::getline(file, line)) {
        ++line_counter;
        if (line_counter > 18) break; // Only take up to the 21st line (lines 4-21)
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> columns;
        while (std::getline(ss, value, ',')) {
            columns.push_back(value);
        }
        if (columns.size() >= 18) {
            y3.push_back(std::stod(columns[2]));
            y6.push_back(std::stod(columns[5]));
            y9.push_back(std::stod(columns[8]));
            y12.push_back(std::stod(columns[11]));
            y15.push_back(std::stod(columns[14]));
            y18.push_back(std::stod(columns[17]));
        }
    }
    file.close();

    std::vector<double> y3_err, y6_err, y9_err, y12_err, y15_err, y18_err;
    for (size_t i = 0; i < y3.size(); ++i) {
        y3_err.push_back( std::sqrt(y3[i]) / 50.0 );
        y6_err.push_back( std::sqrt(y6[i]) / 50.0 );
        y9_err.push_back( std::sqrt(y9[i]) / 50.0 );
        y12_err.push_back( std::sqrt(y12[i]) / 50.0 );
        y15_err.push_back( std::sqrt(y15[i]) / 50.0 );
        y18_err.push_back( std::sqrt(y18[i]) / 50.0 );
    }


    // Create histograms for each dataset
    TH1F* h3  = new TH1F("h3",  "SiArray acceptance;CM Angle (deg);%",  18, 0, 180);
    TH1F* h6  = new TH1F("h6",  "6th Column;Angle (deg);Value",  18, 0, 180);
    TH1F* h9  = new TH1F("h9",  "9th Column;Angle (deg);Value",  18, 0, 180);
    TH1F* h12 = new TH1F("h12", "12th Column;Angle (deg);Value", 18, 0, 180);
    TH1F* h15 = new TH1F("h15", "15th Column;Angle (deg);Value", 18, 0, 180);
    TH1F* h18 = new TH1F("h18", "18th Column;Angle (deg);Value", 18, 0, 180);

     // Fill histograms and set errors
    for (size_t i = 0; i < y3.size() && i < 18; ++i) {
        h3->SetBinContent(i + 1, y3[i]/50.0);
        h3->SetBinError(i + 1, y3_err[i]);
        h6->SetBinContent(i + 1, y6[i]/50.0);
        h6->SetBinError(i + 1, y6_err[i]);
        h9->SetBinContent(i + 1, y9[i]/50.0);
        h9->SetBinError(i + 1, y9_err[i]);
        h12->SetBinContent(i + 1, y12[i]/50.0);
        h12->SetBinError(i + 1, y12_err[i]);
        h15->SetBinContent(i + 1, y15[i]/50.0);
        h15->SetBinError(i + 1, y15_err[i]);
        h18->SetBinContent(i + 1, y18[i]/50.0);
        h18->SetBinError(i + 1, y18_err[i]);
    }

    // Set different colors for each histogram
    h3->SetLineColor(kRed);
    h6->SetLineColor(kBlue);
    h9->SetLineColor(kGreen+2);
    h12->SetLineColor(kMagenta);
    h15->SetLineColor(kOrange+7);
    h18->SetLineColor(kBlack);

    // Draw all histograms on the same canvas
    TCanvas* c = new TCanvas("c", "All Columns", 800, 600);
    h3->Draw("E hist");
    h6->Draw("E hist same");
    h9->Draw("E hist same");
    h12->Draw("E hist same");


    // Add a legend
    auto legend = new TLegend(0.7,0.7,0.9,0.9);
    legend->AddEntry(h3,  "13Be g.s.",  "l");
    legend->AddEntry(h6,  "13Be* 0.4 MeV",  "l");
    legend->AddEntry(h9,  "13Be* 1.6 MeV",  "l");
    legend->AddEntry(h12, "13Be* 2.5 MeV", "l");

    legend->Draw();
}