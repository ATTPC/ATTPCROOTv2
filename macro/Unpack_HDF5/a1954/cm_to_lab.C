#include <fstream>
#include <iostream>
#include <string>

void multiply_first_column() {
    // Specify the input and output file names
    std::string input_filename = "ex_ener_90_data.txt"; // Specify the name of the input file
    std::string output_filename = "ex_ener_90_data_lab_alpha.txt"; // Specify the name of the output file
    Double_t m_a = 4.00260325415 * 931.49401;
    Double_t m_Be10 = 10.013533818 * 931.49401;
    std::ifstream infile(input_filename);
    if (!infile.is_open()) {
        std::cerr << "Error: Could not open file " << input_filename << std::endl;
        return;
    }

    std::ofstream outfile(output_filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open file " << output_filename << std::endl;
        return;
    }

    double col1, col2, col3, col4;

    while (infile >> col1 >> col2 >> col3 >> col4) {
       
       col1 = (col1 * 7.0/5.0);
       col2 = 68;
       col3 = col3 * 0.0005*TMath::Sin(80.0*TMath::DegToRad());
       col4 = col4 * 0.0005*TMath::Sin(80.0*TMath::DegToRad());
       /* double A = 2*m_a*col1*7.0/2.0 + (m_a + m_Be10)*(m_a + m_Be10) + m_a*m_a - m_Be10*m_Be10;

        double a = m_a*m_a;
        double b = m_Be10*m_Be10*m_a - m_a*m_a*m_a - m_a + A*m_a;
        double c = (A*A)/(-4.0);
        col1 = (-b + TMath::Sqrt(b*b - 4.0*a*c))/(2.0*a);
        //col1 = ((0.25*A*A -(m_a*m_a*m_Be10*m_Be10))/(A + m_a*m_a + m_Be10*m_Be10) + m_a*m_a)/(m_a);*/
        outfile << col1 << " " << col2 << " " << col3 << " " << col4 << std::endl;
    }

    infile.close();
    outfile.close();
}

// Example usage
int cm_to_lab() {
    multiply_first_column();
    return 0;
}