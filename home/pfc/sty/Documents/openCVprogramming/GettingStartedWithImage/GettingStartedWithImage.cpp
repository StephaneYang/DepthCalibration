//! [includes]
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

using namespace cv;
//! [includes]

int main()
{
    //! [imread]
    std::string image_path = samples::findFile("/home/pfc/sty/Desktop/ChArUco/images/left/left_1.png");
    int i = 1;
    Mat img = imread(image_path, IMREAD_COLOR);
    //! [imread]

    //! [empty]
    if(img.empty())
    {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return 1;
    }
    //! [empty]
    
    while(!img.empty()) {
        cv::Mat imageCopy;
        img.copyTo(imageCopy);
        cv::imshow("out", imageCopy);
        int k = waitKey(1);
        
        i++;
        image_path = "/home/pfc/sty/Desktop/ChArUco/images/left/left_"+std::__cxx11::to_string(i)+".png";
        img = cv::imread(image_path, cv::IMREAD_COLOR);
    }    

    //! [imshow]
    //imshow("Display window", img);
    //int k = waitKey(0); // Wait for a keystroke in the window
    //! [imshow]

    //! [imsave]
    /*if(k == 's')
    {
        imwrite("starry_night.png", img);
    }*/
    //! [imsave]

    return 0;
}
