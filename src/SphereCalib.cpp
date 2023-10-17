#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>


int main()
{
    {
        //Point Cloud
        const char* xyzFileName = "input/L1_S5.xyz";

        // Open the .xyz file for reading
        std::ifstream inputFile(xyzFileName);

        if (!inputFile.is_open()) {
            std::cerr << "Failed to open the .xyz file." << std::endl;
            return 1;
        }

        // Read and print the first 10 lines
        for (int i = 0; i < 10; i++) {
            double x, y, z;
            inputFile >> x >> y >> z;
            std::cout << "Line " << i + 1 << ": " << x << " " << y << " " << z << std::endl;
        }

        // Close the file
        inputFile.close();
    }
    
    {
        //Image
        std::string image_path =
            "input/C1_S5.bmp";
        cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);

        if (img.empty())
        {
            std::cout << "Could not read the image: " << image_path << std::endl;
            return 1;
        }

        cv::imshow("Display window", img);

        int k = cv::waitKey(0); // Wait for a keystroke in the window
        if (k == 's')
        {
            cv::imwrite("starry_night.png", img);
        }
    }


    return 0;
}
