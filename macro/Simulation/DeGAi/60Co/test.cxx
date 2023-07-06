#include <iostream>
#include <string>
#include <curl/curl.h>

size_t CurlWriteCallback(void* contents, size_t size, size_t nmemb, std::string* output)
{
    size_t totalSize = size * nmemb;
    output->append((char*)contents, totalSize);
    return totalSize;
}

int test()
{
    CURL* curl = curl_easy_init();
    if (curl)
    {
        std::string data;
        std::string isotope = "Am241";  // Example isotope

        std::string url = "https://www.nndc.bnl.gov/nudat2/decaysearchdirect.jsp?nuc=" + isotope;
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, CurlWriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &data);
        curl_easy_perform(curl);

        // Process and extract the photpeak values from the retrieved data
        // (You'll need to implement the parsing logic based on the specific format of the data)

        // Print out the photpeak values
        std::cout << "Photpeak values for isotope " << isotope << ":\n";
        // Iterate over the values and print them out
        // ...

        curl_easy_cleanup(curl);
    }

    return 0;
}

