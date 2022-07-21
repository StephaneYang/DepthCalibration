//! [charucohdr]
#include <opencv2/aruco/charuco.hpp>
//! [charucohdr]
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>

namespace {
const char* about = "A tutorial code on charuco board creation and detection of charuco board with and without camera caliberation";
const char* keys = "{c        |       | Put value of c=1 to create charuco board;\nc=2 to detect charuco board without camera calibration;\nc=3 to detect charuco board with camera calibration and Pose Estimation}";
}

void createBoard();
void detectCharucoBoardWithCalibrationPose();
void detectCharucoBoardWithoutCalibration();

static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

void createBoard()
{
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    //! [createBoard]
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(9600, 8000), boardImage, 10, 1);
    //! [createBoard]
    cv::imwrite("BoardImage.png", boardImage);
}

//! [detwcp]
void detectCharucoBoardWithCalibrationPose()
{
    std::string image_path = "/home/pfc/sty/Desktop/ChArUco/images/left/left_1.png";
    int i = 1;
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    //! [matdiscoff]
    cv::Mat cameraMatrix, distCoeffs;
    std::string filename = "calib.txt";
    bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
    //! [matdiscoff]
    if (!readOk) {
        std::cerr << "Invalid camera file" << std::endl;
    } else {
        //! [dictboard]
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
        cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
        //! [dictboard]
        while (!img.empty()) {
            cv::Mat imageCopy;
            img.copyTo(imageCopy);
            //! [midcornerdet]
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f> > markerCorners;
            cv::aruco::detectMarkers(img, board->dictionary, markerCorners, markerIds, params);
            //! [midcornerdet]
            // if at least one marker detected
            if (markerIds.size() > 0) {
                cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
                //! [charidcor]
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, img, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
                //! [charidcor]
                // if at least one charuco corner detected
                if (charucoIds.size() > 0) {
                    cv::Scalar color = cv::Scalar(255, 0, 0);
                    //! [detcor]
                    cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
                    //! [detcor]
                    cv::Vec3d rvec, tvec;
                    //! [pose]
                    // cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    //! [pose]
                    bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    // if charuco pose is valid
                    if (valid)
                        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1f);
                }
            }
            cv::imshow("out", imageCopy);
            char key = (char)cv::waitKey(30);
            if (key == 27)
                break;
            
            i++;
            image_path = "/home/pfc/sty/Desktop/ChArUco/images/left/left_"+std::__cxx11::to_string(i)+".png";
            img = cv::imread(image_path, cv::IMREAD_COLOR);
        }
    }
}
//! [detwcp]

//! [detwc]
void detectCharucoBoardWithoutCalibration()
{
    std::string image_path = "/home/pfc/sty/Desktop/ChArUco/images/left/left_1.png";
    int i = 1;
    int tab [24];
    for (int ii=0; ii<24; ii++)
    {
        tab[ii] = 0;
    }
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);

    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    while (!img.empty()) {
        std::cout << "charuco_"+std::__cxx11::to_string(i)+".png" << std::endl;
        
        cv::Mat imageCopy;
        img.copyTo(imageCopy);
        
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::aruco::detectMarkers(img, board->dictionary, markerCorners, markerIds, params);
        //or
        //cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, params);
        // if at least one marker detected
        if (markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
            //! [charidcorwc]
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, img, board, charucoCorners, charucoIds);
            //! [charidcorwc]
            // if at least one charuco corner detected
            if (charucoIds.size() > 0)
                cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
            
            
            for (auto ii: charucoIds)
            {
                std::cout << ii << " ";
                tab[ii]++;
            }
            std::cout << std::endl;
            for (auto ii: charucoCorners)
            {
                std::cout << ii.x << " " <<ii.y << std::endl;
            }
            std::cout << std::endl;
        }

        cv::imshow("out", imageCopy);
        //cv::imwrite("charuco_"+std::__cxx11::to_string(i)+".png", imageCopy);
        char key = (char)cv::waitKey(1);
        if (key == 27)
            break;
            
        i++;
        image_path = "/home/pfc/sty/Desktop/ChArUco/images/left/left_"+std::__cxx11::to_string(i)+".png";
        img = cv::imread(image_path, cv::IMREAD_COLOR);
    }
    for (int ii=0; ii<24; ii++)
    {
        std::cout << "Id " + std::__cxx11::to_string(ii) + " apppeared " + std::__cxx11::to_string(tab[ii]) + " times" << std::endl;
    }
}
//! [detwc]

int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);
    if (argc < 2) {
        parser.printMessage();
        return 0;
    }
    int choose = parser.get<int>("c");
    
    switch (choose) {
    case 1:
        createBoard();
        std::cout << "An image named BoardImg.jpg is generated in folder containing this file" << std::endl;
        break;
    case 2:
        detectCharucoBoardWithoutCalibration();
        break;
    case 3:
        detectCharucoBoardWithCalibrationPose();
        break;
    default:
        break;
    }
    return 0;
}
