//left to right : lines 612-619, 658
#include <memory>
#include <iostream>
#include <string>

// JSON Parser include
#include "json_parser.hpp"
#include "ptree.hpp"

// ZED include
#include <sl/Camera.hpp>

// ChArUco include
#include <opencv2/aruco/charuco.hpp>

//Eigen include
#include <Eigen/Dense>
#include <Eigen/SVD>

// Sample includes
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "utils.hpp"

#define maxFrame 1000 // 1000 is the maximum number of images we allow

// Using namespace
using namespace sl;
using namespace std;
namespace pt = boost::property_tree;
using Eigen::MatrixXd;

void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");
void SaveFramesFromSVO(int svo_position);

void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix)
{
	cout <<"[Sample]";
	if (err_code != ERROR_CODE::SUCCESS)
		cout << "[Error] ";
	else
		cout<<" ";
	cout << msg_prefix << " ";
	if (err_code != ERROR_CODE::SUCCESS)
	{
		cout << " | " << toString(err_code) << " : ";
		cout << toVerbose(err_code);
	}
	if (!msg_suffix.empty())
		cout << " " << msg_suffix;
	cout << endl;
}

void SaveFramesFromSVO(sl::Mat svo_image, sl::Mat depth_for_display, sl::Mat imageZED, int svo_position)
{
	// Save the color display (LEFT & RIGHT) & the depth display & the other color display (LEFT ONLY)
	svo_image.write(("/home/pfc/sty/Documents/cameraCalib/build/images/capture_" + to_string(svo_position) + ".png").c_str());
	depth_for_display.write(("/home/pfc/sty/Documents/cameraCalib/build/depths/depth_" + to_string(svo_position) + ".png").c_str());
	imageZED.write(("/home/pfc/sty/Documents/cameraCalib/build/images/left/left_" + to_string(svo_position) + ".png").c_str());
}

class JSON
{
	public:
		// Create a tree
		pt::ptree jsonFile;
		int totalNumber = 0; // Total number of views
		int key2img[maxFrame]; // Correspondence between key (input) and image number (output)
		int img2key[maxFrame]; // Correspondence between image number (input) and key (output)
		double focal_length = 1400; // Focal length
		double principal_point[2]; // Coordinates of the principal point
		double disto_k3[3]; // Lense distortion
		double center[maxFrame][3]; // The center of each view
		double rotation[maxFrame][9]; // The rotation matrix in an array (reading left to right, up to down) of each view
		
		JSON(const std::string & filename) // Constructor
		{
			pt::read_json(filename, jsonFile);  // Load the json file in this ptree
			int i = 0, j = 0;
			
			for(auto&& v : jsonFile.get_child("views"))
			{
				const pt::ptree & tree = v.second; // blank key associated to this subtree due to rectangular bracket (see JSON Parser from boost's ptree : https://www.boost.org/doc/libs/1_79_0/doc/html/property_tree/parsers.html#property_tree.parsers.json_parser)
		
				if(tree.empty())
				{
					//std::cout << "tree empty" << std::endl;
				}
				else
				{
					//std::cout << "tree not empty" << std::endl;

					for(auto&& sv : tree) // inside blank key's subtree
					{
						const std::string & key = sv.first; // "key", "value"
						const pt::ptree & subtree = sv.second;
						//std::cout << key << std::endl;
						if(subtree.empty())
						{
						//std::cout << "subtree empty" << std::endl;
						//std::cout << subtree.get_value_optional<int>() << endl; // gives the value of "key"
						}
						else
						{
							//std::cout << "subtree not empty (value)" << std::endl;

							for(auto&& ssv: subtree) // inside "value"
							{
								const std::string & subkey = ssv.first; // "polymorphic_id", "ptr_wrapper"
								const pt::ptree & subsubtree = ssv.second;
								//std::cout << subkey << std::endl;
								if(subsubtree.empty())
								{
									//std::cout << "subsubtree empty" << std::endl;
									//std::cout << subsubtree.get_value_optional<int>() << endl;
								}
								else
								{
									//std::cout << "subsubtree not empty (ptr_wrapper)" << std::endl;

									for(auto&& sssv: subsubtree) // inside "ptr_wrapper"
									{
										const std::string & subsubkey = sssv.first; // "id", "data"
										const pt::ptree & subsubsubtree = sssv.second;
										//std::cout << subsubkey << std::endl;
										if(subsubsubtree.empty())
										{
											//std::cout << "subsubsubtree empty" << std::endl;
											//std::cout << subsubsubtree.get_value_optional<string>() << endl;
										}
										else
										{
											//std::cout << "subsubtree not empty (data)" << std::endl;

											for(auto&& ssssv: subsubsubtree) // inside "data"
											{
												const std::string & subsubsubkey = ssssv.first; // "local_path", "filename", "width", "height", "id_view", "id_intrinsic", "id_pose"
												const pt::ptree & subsubsubsubtree = ssssv.second;
												
												const std::string leaf = subsubsubsubtree.get_value<string>();
												
												//std::cout << subsubsubkey << endl;
												//std::cout << subsubsubsubtree.get_value<string>() << endl;
												if(subsubsubkey == "filename")
												{
													//int res = leaf.length() - 10;
													//std::string imgNum = leaf.substr(5, 1+res);
													//int imgNumber = std::stoi(imgNum);
													int imgNumber = std::stoi(leaf.substr(5, leaf.length() - 9));
													//std::cout << imgNumber << endl;
													key2img[i] = imgNumber;
													img2key[imgNumber] = i;
													i++;
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
			
			totalNumber = i;
			i = 0;
			
			for(int i=0; i<totalNumber; i++)
			{
				for(int j=0; j<3; j++)
					center[i][j] = -1;
					
				for(int j=0; j<9; j++)
					rotation[i][j] = -1;
			}
			
			for(auto&& v : jsonFile.get_child("intrinsics"))
			{
				const pt::ptree & tree = v.second; // blank key associated to this subtree (rect bracket parsing)
				
				if(!tree.empty())
				{
					for(auto&& sv : tree) // inside blank key's subtree
					{
						const std::string & key = sv.first; // "key", "value"
						const pt::ptree & subtree = sv.second;
						
						if(!subtree.empty())
						{
							for(auto&& ssv: subtree) // inside "value"
							{
								const std::string & subkey = ssv.first; // "polymorphic_id", "polymorphic_name", "ptr_wrapper"
								const pt::ptree & subsubtree = ssv.second;
								
								if(!subsubtree.empty())
								{
									for(auto&& sssv: subsubtree) // inside "ptr_wrapper"
									{
										const std::string & subsubkey = sssv.first; // "id", "data"
										const pt::ptree & subsubsubtree = sssv.second;
										
										if(!subsubsubtree.empty())
										{
											for(auto&& ssssv: subsubsubtree) // inside "data"
											{
												const std::string & subsubsubkey = ssssv.first; // "width", "height", "focal_length", "principal_point", "disto_k3"
												const pt::ptree & subsubsubsubtree = ssssv.second;
												
												if(subsubsubkey == "focal_length")
												{
													focal_length = subsubsubsubtree.get_value<double>();
												}
												
												if(!subsubsubsubtree.empty())
												{
													for(auto&& sssssv: subsubsubsubtree) // inside "principal_point" OR "disto_k3"
													{
														double leaf = sssssv.second.get_value<double>();
														
														if(subsubsubkey == "principal_point") // inside "principal_point"
														{
															principal_point[j] = leaf;
														}
														
														if(subsubsubkey == "disto_k3") // inside "disto_k3"
														{
															disto_k3[j] = leaf;
														}
														j++;
													}
													j = 0;
												}
											}
										}
									}
								}
							}
						}
					}
				}
				i++;
			}
			
			for(auto&& v : jsonFile.get_child("extrinsics"))
			{
				const pt::ptree & tree = v.second; // blank key associated to this subtree (rect bracket parsing)
				
				if(!tree.empty())
				{
					for(auto&& sv : tree) // inside blank key's subtree
					{
						const std::string & key = sv.first; // "key", "value"
						const pt::ptree & subtree = sv.second;
						if(key == "key")
						{
							i = sv.second.get_value<int>();
						}
						
						if(!subtree.empty())
						{
							for(auto&& ssv: subtree) // inside "value"
							{
								const std::string & subkey = ssv.first; // "rotation", "center"
								const pt::ptree & subsubtree = ssv.second;
								
								if(!subsubtree.empty())
								{
									for(auto&& sssv: subsubtree) // inside "rotation" OR "center"
									{
										const pt::ptree & subsubsubtree = sssv.second;
										
										if(subsubsubtree.empty()) // This should be "center"
										{
											center[i][j] = subsubsubtree.get_value<double>();
											j++;
										}
										else // This should be "rotation"
										{
											for(auto&& ssssv: subsubsubtree) // inside "rotation"
											{
												rotation[i][j] = ssssv.second.get_value<double>();
												j++;
											}
										}
									}
									j = 0;
								}
							}
						}
					}
				}
			}
		}
};

class CHARUCO
{
	public:
		double x[maxFrame][24]; // There are 24 ChArUco Ids and each represents a corner
		double y[maxFrame][24];
		int done = 0; // Tell if ChArUco detection is already done so we don't have to do it again
		
		CHARUCO()
		{
			int i, j;
			for(i=0; i<maxFrame; i++)
			{
				for(j=0; j<24; j++)
				{
					x[i][j] = -1;
					y[i][j] = -1;
				}
			}
		}
		
		void start()
		{
			int i = 0, j = 0;
			int check[24]; // Checklist to check which ChArUco is detected
			
			vector<cv::String> fn;
			cv::glob("/home/pfc/sty/Documents/cameraCalib/build/images/left/*.png", fn, false);

			vector<cv::Mat> images;
			int count = fn.size(); //number of png files in images folder

			cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
			cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
			params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
			for (int k=0; k<count; k++)
			{
				for (j=0; j<24; j++)
				{
					check[j] = 0;
				}
				images.push_back(cv::imread(fn[k]));
				cv::Mat imageCopy;
				images[k].copyTo(imageCopy);
				i = std::stoi(fn[k].substr(59, fn[k].length() - 63));

				std::vector<int> markerIds;
				std::vector<std::vector<cv::Point2f> > markerCorners;
				cv::aruco::detectMarkers(images[k], board->dictionary, markerCorners, markerIds, params);
				// if at least one marker detected
				if(markerIds.size() > 0)
				{
					cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
					//! [charidcorwc]
					std::vector<cv::Point2f> charucoCorners;
					std::vector<int> charucoIds;
					cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, images[k], board, charucoCorners, charucoIds);
					//! [charidcorwc]
					// if at least one charuco corner detected
					if(charucoIds.size() > 0)
					{
						cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
						for(auto id: charucoIds)
							check[id] = 1;

						for(auto corner: charucoCorners)
						{
							int stop = 0;
							int ii = 0;
							while(stop == 0)
							{
								if(check[ii] == 1)
								{
									x[i][ii] = corner.x;
									y[i][ii] = corner.y;
									check[ii] = 0;
									stop = 1;
								}
								ii++;
								if (ii >= 24)
									stop = 1;
							}
						}
					}
				}
			/*cv::imshow("out", imageCopy);
			char key = (char)cv::waitKey(1);
			if (key == 27)
				break;*/
			}
			done = 1;
		}
		
		void createBoard()
		{
			cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
			cv::Mat boardImage;
			board->draw(cv::Size(2400, 2000), boardImage, 10, 1);
			//! [createBoard]
			cv::imwrite("/home/pfc/sty/Documents/cameraCalib/build/charuco/BoardImage.png", boardImage);
		}
		
		void save()
		{
			int i = 0;
			
			vector<cv::String> fn;
			cv::glob("/home/pfc/sty/Documents/cameraCalib/build/images/left/*.png", fn, false);

			vector<cv::Mat> images;
			int count = fn.size(); //number of png files in images folder
			
			cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
			cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
			cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
			params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
			for(int k=0; k<count; k++)
			{
				images.push_back(cv::imread(fn[k]));
				cv::Mat imageCopy;
				images[k].copyTo(imageCopy);
				i = std::stoi(fn[k].substr(59, fn[k].length() - 63));

				std::vector<int> markerIds;
				std::vector<std::vector<cv::Point2f> > markerCorners;
				cv::aruco::detectMarkers(images[k], board->dictionary, markerCorners, markerIds, params);
				// if at least one marker detected
				if(markerIds.size() > 0)
				{
					cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
					//! [charidcorwc]
					std::vector<cv::Point2f> charucoCorners;
					std::vector<int> charucoIds;
					cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, images[k], board, charucoCorners, charucoIds);
					//! [charidcorwc]
					// if at least one charuco corner detected
					if(charucoIds.size() > 0)
						cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
				}
			cv::imwrite("/home/pfc/sty/Documents/cameraCalib/build/charuco/charuco_"+to_string(i)+".png", imageCopy);
			}
		}
};

MatrixXd FeaturePoint3Dposition(const MatrixXd&, JSON, CHARUCO, int);

MatrixXd FeaturePoint3Dposition(const MatrixXd& Intrinsic, JSON ptitJson, CHARUCO charc, int id)
{
	string txt = "";
	txt = txt.append("List of points which follow the line between the corresponding camera center and the selected id : \n They are farther so we can draw a longer line \n\n x y z R G B related_to_which_key_view(which camera center)\n");
	
	MatrixXd Identity = MatrixXd::Identity(3,3); 

	MatrixXd R(3,3); // Matrix composing the least-square equation
	R = MatrixXd::Zero(3, 3);

	MatrixXd q(3, 1); // Vector also composing the equation
	q = MatrixXd::Zero(3, 1);
	
	MatrixXd secondPoint(3, 1); // Second point to trace a line between the camera center and where it is looking (optional)
	secondPoint = MatrixXd::Zero(3, 1);
	
	for (int temp = 0; temp < ptitJson.totalNumber; temp++)
	{
		cout << endl << "key : " << temp << endl;
		int imnb = ptitJson.key2img[temp]; // temp will run through all the view keys, so imnb will correspond to image number
		cout << "left_" << imnb << ".png" << endl;

		MatrixXd p2D(2,1); // Feature point of the current frame (camera coordinates)
		p2D(0,0) = charc.x[imnb][id];
		p2D(1,0) = charc.y[imnb][id];
		cout << "ChArUco feature point in 2D :" << endl << p2D << endl;
		
		if ((ptitJson.center[temp][0] != -1)&&(charc.x[imnb][id] != -1))
		{
			MatrixXd camOrigin(3,1); // Origins of the camera
			camOrigin(0,0) = ptitJson.center[temp][0];
			camOrigin(1,0) = ptitJson.center[temp][1];
			camOrigin(2,0) = ptitJson.center[temp][2];
			cout << endl << "Origins of the camera :" << endl << camOrigin << endl;

			MatrixXd Rotation(3,3); // Rotation matrix (world->camera oriented)
			Rotation(0,0) = ptitJson.rotation[temp][0];
			Rotation(0,1) = ptitJson.rotation[temp][1];
			Rotation(0,2) = ptitJson.rotation[temp][2];
			Rotation(1,0) = ptitJson.rotation[temp][3];
			Rotation(1,1) = ptitJson.rotation[temp][4];
			Rotation(1,2) = ptitJson.rotation[temp][5];
			Rotation(2,0) = ptitJson.rotation[temp][6];
			Rotation(2,1) = ptitJson.rotation[temp][7];
			Rotation(2,2) = ptitJson.rotation[temp][8];
			cout << endl << "Rotation matrix :" << endl << Rotation << endl;

			MatrixXd RotationT(3,3); // Rotation matrix (camera oriented->world) coordinates
			RotationT = Rotation.transpose();

			MatrixXd Direction(3,1); // Direction vector (in camera coordinates)
			Direction(0,0) = -(Intrinsic(0,2) - p2D(0,0));
			Direction(1,0) = -(Intrinsic(1,2) - p2D(1,0));
			Direction(2,0) = Intrinsic(0,0);
			cout << endl << "prinpal0 " << Intrinsic(0,2) << endl;
			cout << "px " << p2D(0,0) << endl;
			cout << "prinpal1 " << Intrinsic(1,2) << endl;
			cout << "py " << p2D(1,0) << endl;
			cout << "Camera direction :" << endl << Direction << endl;
			Direction = RotationT*Direction; // In world coordinates
			cout << endl << "World direction :" << endl << Direction << endl;
			Direction = Direction/Direction.norm(); // Normalize
			cout << endl << "Normalized direction :" << endl << Direction << endl;

			R = R + (Identity - Direction*Direction.transpose());
			q = q + (Identity - Direction*Direction.transpose())*camOrigin;
			
			secondPoint = camOrigin + 10*Direction;
			txt = txt.append(to_string(secondPoint(0,0))+" "+to_string(secondPoint(1,0))+" "+to_string(secondPoint(2,0))+" 0 0 255 "+to_string(temp)+"\n");
		}
		else
		{
			if (ptitJson.center[temp][0] == -1)
				cout << "bad image, no camera pose detected" << endl;
			if (charc.x[imnb][id] == -1)
				cout << "bad image, no charuco pose detected" << endl;
		}
	}
	cout << endl << "R" << endl << R << endl;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
	cout << "Its singular values are:" << endl << svd.singularValues() << endl;

	cout << "q" << endl << q << endl;

	std::ofstream outfile ("Id"+to_string(id)+".txt");
	outfile << txt << std::endl;
	outfile.close();

	return svd.solve(q);
}

int main(int argc, char **argv)
{
	if (system("mv /home/pfc/sty/Documents/cameraCalib/build/temporaryFolder/* /home/pfc/sty/Documents/cameraCalib/build/images/left/ ") == -1);
	if (argc<=1)
	{
		cout << "Usage: \n";
		cout << "$ ZED_SVO_Playback <SVO_file> \n";
		cout << "  ** SVO file is mandatory in the application ** \n\n";
		return EXIT_FAILURE;
	}

	// Create ZED objects
	Camera zed;
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE::STANDARD; // Set sensing mode in (FILL or STANDARD)
	InitParameters init_parameters;
	init_parameters.coordinate_units = UNIT::METER; // Use meter units
	init_parameters.input.setFromSVOFile(argv[1]);
	init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA; // Set the depth mode to ultra
	init_parameters.depth_minimum_distance = 0.10 ; // Set the minimum depth perception distance to 10 cm
	init_parameters.depth_maximum_distance = 40; // Set the maximum depth perception distance to 40m

	// Create CHARUCO object
	CHARUCO charc;

	// Open the camera
	auto returned_state = zed.open(init_parameters);
	if (returned_state != ERROR_CODE::SUCCESS)
	{
		print("Camera Open", returned_state, "Exit program.");
		return EXIT_FAILURE;
	}
	
	auto resolution = zed.getCameraInformation().camera_configuration.resolution;
	// Define OpenCV window size (resize to max 720/404)
	// for left & right (for display)
	sl::Resolution low_resolution(min(720, (int)resolution.width) * 2, min(404, (int)resolution.height));
	sl::Mat svo_image(low_resolution, MAT_TYPE::U8_C4, MEM::CPU);
	cv::Mat svo_image_ocv = slMat2cvMat(svo_image);
	// and for depth (for display)
	sl::Resolution low_resolution_depth(min(720, (int)resolution.width), min(404, (int)resolution.height));
	sl::Mat depth_for_display(low_resolution_depth, MAT_TYPE::U8_C4, MEM::CPU);
	cv::Mat depth_for_display_ocv = slMat2cvMat(depth_for_display);
	// and just for left (for SfM pipeline and depth calculation)
	sl::Mat imageZED(zed.getCameraInformation().camera_resolution.width, zed.getCameraInformation().camera_resolution.height, MAT_TYPE::U8_C4);

	// Setup key, images, times
	char key = ' ';
	cout << " Press 'f' to jump forward in the video (optional)" << endl;
	cout << " Press 'b' to jump backward in the video (optional)" << endl;
	cout << " Press 'w' to wash everything" << endl;
	cout << " Press 's' to save the video as an image list in all different displays" << endl;
	cout << " Press 'c' to save the images from the ChArUco detection (optional but always after s)" << endl;
	cout << " Press 'p' to launch the SFM pipeline" << endl;
	cout << " Press 'd' to compare the depth given by the ZED camera software and the depth calculation using all the previous data" << endl;
	cout << " Press 'q' or 'esc' to exit..." << endl;
	cout << " Note that the order should be : w -> s -> p -> d , the rest are optional" << endl;

	// Variables for ZED camera
	int svo_frame_rate = zed.getInitParameters().camera_fps;
	int nb_frames = zed.getSVONumberOfFrames();
	print("[Info] SVO contains "+to_string(nb_frames)+" frames");

	// Variables for frames and depth calculation
	int saveIncrement = 0;
	int saveOK = 0;
	sl::Mat depth, point_cloud;
	
	// Calibration parameters (focal distance, principal point's coordinates, lens distortions)
	double focal_left_x = 1400; // Focal distance by default (given by the datasheet of the ZED camera)
	double principal_point_x = 960; // In the middle by default
	double principal_point_y = 540;
	double k1, k2, k3; // Radial distortions
	double p1, p2; // Tangential distortions
	auto camera_infos = zed.getCameraInformation(); // Get camera information
	
	focal_left_x = (double)camera_infos.camera_configuration.calibration_parameters.right_cam.fx; // Real focal length of the left eye in pixels stored in getCameraInformation()
	principal_point_x = (double)camera_infos.camera_configuration.calibration_parameters.right_cam.cx;
	principal_point_y = (double)camera_infos.camera_configuration.calibration_parameters.right_cam.cy;
	k1 = camera_infos.camera_configuration.calibration_parameters.right_cam.disto[0];
	k2 = camera_infos.camera_configuration.calibration_parameters.right_cam.disto[1];
	p1 = camera_infos.camera_configuration.calibration_parameters.right_cam.disto[2];
	p1 = camera_infos.camera_configuration.calibration_parameters.right_cam.disto[3];
	k3 = camera_infos.camera_configuration.calibration_parameters.right_cam.disto[4];
	cout << "focal_left_x " << focal_left_x << endl;
	cout << "principal_point_x " << principal_point_x << endl;
	cout << "principal_point_y " << principal_point_y << endl;
	cout << "k1 " << k1 << endl << "k2 " << k2 << endl << "k3 " << k3 << endl << "p1 " << p1 << endl << "p2 " << p2 << endl;
	
	string depth_txt = ""; // text to store the depths

	// Start SVO playback

	while ((key != 'q')&&(key != 27)) // not 'q' nor 'esc'
	{
		returned_state = zed.grab();
		if (returned_state == ERROR_CODE::SUCCESS)
		{
			int svo_position = zed.getSVOPosition(); // From 0 to nb_frames-1
			
			if ((saveIncrement < nb_frames)&&(saveOK == 1)) // we only need to get depth and save the images in one loop (that's why 'saveIncrement' will not go back to 0)
			{
				SaveFramesFromSVO(svo_image, depth_for_display, imageZED, svo_position);
				saveIncrement++;
			}
			
			if (saveIncrement == nb_frames)
			{
				cout << "Saving DONE!" << endl;
				cout << "Executing ChArUco detection, please wait a minute" << endl;
				charc.start();
				cout << "Executing ChArUco detection, DONE" << endl;
				saveIncrement++; // Just so that save Increment is no longer equal to nb_frames
			}

			// Get the side by side image
			zed.retrieveImage(svo_image, VIEW::SIDE_BY_SIDE, MEM::CPU, low_resolution);

			// Get the depth for display
			zed.retrieveImage(depth_for_display,VIEW::DEPTH, MEM::CPU, low_resolution_depth);

			// Get the left side image
			zed.retrieveImage(imageZED, VIEW::RIGHT);

			// Display the frame (LEFT&RIGHT) & depth
			cv::imshow("View_L&R", svo_image_ocv);
			cv::imshow("View_D", depth_for_display_ocv);
			key = cv::waitKey(10);

			switch (key)
			{
				case 's':
				{
					// Activate image saving (all the frames will be saved)
					saveOK = 1;
					zed.setSVOPosition(0);
					key = cv::waitKey(300);
					cout << "Saving..." << endl;
					break;
				}
				case 'f':
				{
					zed.setSVOPosition(svo_position + svo_frame_rate);
					break;
				}
				case 'b':
				{
					zed.setSVOPosition(svo_position - svo_frame_rate);
					break;
				}
				case 'p':
				{
					// Charuco detection if not done
					if (charc.done != 1)
					{
						cout << "Executing ChArUco detection, please wait a minute" << endl;
						charc.start();
						cout << "Executing ChArUco detection, DONE" << endl;
					}
							
							
					// Remove the images that contains no charuco detection
					for (int imnb = 1; imnb <= nb_frames; imnb++)
					{
						int remove = 1;
						for (int id = 0; id < 24; id++)
						{
							if (charc.x[imnb][id] != -1) // At least one id is detected
								remove = 0; // So we don't move the image
						}
						if (remove == 1) // Not a single id is detected
							if (system(("mv /home/pfc/sty/Documents/cameraCalib/build/images/left/left_"+to_string(imnb)+".png /home/pfc/sty/Documents/cameraCalib/build/temporaryFolder").c_str()) == -1); // Proceed to move them somewhere
					}
					// Send the images (left) for the pose estimation in the SfM pipeline (openMVG)
					cout<<"Run the SfM pipeline"<<endl;
					string cmd ("python3 /home/pfc/sty/openMVG_Build/software/SfM/SfM_GlobalPipeline.py /home/pfc/sty/Documents/cameraCalib/build/images/left/ /home/pfc/sty/Documents/cameraCalib/build/3Dreconstructed/ ");
					string intrinsics; // "f;0;ppx;0;f;ppy;0;0;1"
					intrinsics = to_string(focal_left_x) + " 0 " + to_string(principal_point_x) + " 0 " + to_string(focal_left_x) + " " + to_string((int)principal_point_y) + " 0 0 1";
					string str = cmd + intrinsics;
					const char * c = str.c_str();
					cout<<c<<endl<<endl;
					if(system(c));
					// Return the images
					if (system("mv /home/pfc/sty/Documents/cameraCalib/build/temporaryFolder/* /home/pfc/sty/Documents/cameraCalib/build/images/left/ ") == -1);
					break;
				}
				case 'c':
				{
					// Save the images from the ChArUco detection
					cout << "CHARUCO save" << endl;
					charc.createBoard();
					charc.save();
					cout << "CHARUCO save, DONE" << endl;
					break;
				}
				case 't': // for test
				{
					break;
				}
				case 'd':
				{
					MatrixXd p[24];
					for (int id=0; id<24; id++)
					{
						p[id] = MatrixXd::Zero(3, 1);
					}
				
					MatrixXd Intrinsic(3,3); // Intrinsics matrix
					Intrinsic = MatrixXd::Zero(3, 3);
					
					// Retrieve the data contained in sfm_data.json (views, intrinsics, extrinsics)
					if(system("/home/pfc/sty/openMVG_Build/Linux-x86_64-RELEASE/openMVG_main_ConvertSfM_DataFormat binary -i /home/pfc/sty/Documents/cameraCalib/build/3Dreconstructed/reconstruction_global/sfm_data.bin -o /home/pfc/sty/Documents/cameraCalib/build/3Dreconstructed/reconstruction_global/sfm-data.json -V -I -E") == -1);
					JSON ptitJson("/home/pfc/sty/Documents/cameraCalib/build/3Dreconstructed/reconstruction_global/sfm-data.json");
					
					// Charuco detection if not done
					if (charc.done != 1)
					{
						cout << "Executing ChArUco detection, please wait a minute" << endl;
						charc.start();
						cout << "Executing ChArUco detection, DONE" << endl;
					}
					
					Intrinsic(0,0) = focal_left_x;
					Intrinsic(0,2) = principal_point_x;
					Intrinsic(1,1) = focal_left_x;
					Intrinsic(1,2) = principal_point_y;
					Intrinsic(2,2) = 1;
					cout << endl << "Intrinsics Matrix :" << endl << Intrinsic << endl;
					
					// 3D point calculation
					for (int id=0; id<24; id++)
					{
						p[id] = FeaturePoint3Dposition(Intrinsic, ptitJson, charc, id);
						cout << endl << "point calculated (id:"+to_string(id)+")"<< endl << p[id] << endl;
					}
					
					// Mean distance between two adjacent points in order to find the scale factor and get values in mm
					// Provided that this distance is 40 mm
					double scale = 1.0;
					double mean_distance = 0.0;
					double nb_elem = 0;
					double real_size = 40.0;
					for (int a=0; a<6; a++)
					{
						for (int b=0; b<4; b++)
						{
							MatrixXd differenceb;
							MatrixXd differencea;
							if (b < 3)
							{
								differenceb = p[4*a + b] - p[4*a + b+1];
								cout << "distanceb " << differenceb.norm() << endl;
								mean_distance = mean_distance + differenceb.norm();
								nb_elem++;
							}
							
							if (a < 5)
							{
								differencea = p[4*a + b] - p[4*(a+1) + b];
								cout << "distancea " << differencea.norm() << endl;
								mean_distance = mean_distance + differencea.norm();
								nb_elem++;
							}
						}
					}
					cout << endl << "mean distance is " << mean_distance/nb_elem << endl;
					scale = real_size/(mean_distance/nb_elem);
					cout << "scale is " << scale << endl;
					
					// Save the two depth (one from ZED, one from our calculation)
					for (int id = 0; id < 24; id++)
					{
						depth_txt = "";
						for (int temp = 0; temp < nb_frames; temp++)
						{
							/////////cout << "temp:" << temp << " number of frame:" << nb_frames << endl;
							returned_state = zed.grab();
							int svo_position = zed.getSVOPosition(); // From 1 to nb_frames, correspond to image number
							if (returned_state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
							{
								zed.setSVOPosition(0);
								zed.grab();
								svo_position = zed.getSVOPosition();
							}
							int keynb = ptitJson.img2key[svo_position];
							
							// Get the depth (given by zed softwares)
							zed.retrieveMeasure(depth, MEASURE::DEPTH); // Retrieve depth Mat
							
							// Get a certain feature point's coordinates in pixels (Id : 9, for example)
							int x = (int)(charc.x[svo_position][id]+0.5);
							int y = (int)(charc.y[svo_position][id]+0.5);
							float depth_value;
							depth.getValue(x,y, &depth_value);
							
							MatrixXd camOrigin(3,1); // Origins of the camera
							camOrigin(0,0) = ptitJson.center[keynb][0];
							camOrigin(1,0) = ptitJson.center[keynb][1];
							camOrigin(2,0) = ptitJson.center[keynb][2];
							////////cout << "for img " << svo_position << endl;
							////////cout << "for key " << keynb << endl;
							////////cout << "cam" << endl << camOrigin << endl;
							MatrixXd depth_calculated = camOrigin - p[id];
							//cout << "depth_vector_calculated" << endl << depth_calculated << endl;
							double depth_value_calculated = depth_calculated.norm()*scale;
							////////cout << "depth_calculated " << depth_value_calculated << endl << endl;

							// Check for neighbours coordinates's depth if no value
							int path_x[24] = {1, 1, 0, -1, -1, -1, 0, 1, 2, 2, 2, 1, 0, -1, -2, -2, -2, -2, -2, -1, 0, 1, 2, 2};
							int path_y[24] = {0, 1, 1, 1, 0, -1, -1, -1, 0, 1, 2, 2, 2, 2, 2, 1, 0, -1, -2, -2, -2, -2, -2, -1};
							int path = 0;
							while ((path < 24)&&(!isfinite(depth_value)))
							{
								depth.getValue(x+path_x[path],y+path_y[path], &depth_value);
								path++;
							}
							depth_value = depth_value*1000; // METERS TO MILLIMETERS
							
							if ((x <= 0)||(camOrigin(0,0) == -1)||(!isfinite(depth_value)))
							{
								//depth_txt = depth_txt.append("Image n·"+to_string(svo_position)+"\t"+"depth could not be calculated\t "+ to_string(depth_value)+" pxl depth(ZED)\t at x = "+to_string(x)+", y = "+to_string(y)+"\n");
							}
							else
							{
								depth_txt = depth_txt.append("Image n·"+to_string(svo_position)+"\t"+to_string(depth_value_calculated)+" mm depth(calculated)\t "+ to_string(depth_value)+" mm depth(ZED)\t at x = "+to_string(x)+"-"+to_string(x+path_x[path]-1)+", y = "+to_string(y)+"-"+to_string(y+path_y[path]-1)+"\t difference of "+to_string(depth_value_calculated-depth_value)+" mm"+"\n");
							}
						}

						depth_txt = depth_txt.append("\nlist of charuco Ids :\n");
						for (int idd=0; idd<24; idd++)
						{
							MatrixXd& b = p[idd];
							depth_txt = depth_txt.append(to_string(b(0,0))+" "+to_string(b(1,0))+" "+to_string(b(2,0))+" 255 0 0\n");
						}
						
						std::ofstream outfile ("depthComparisonId"+to_string(id)+".txt");
						outfile << depth_txt << std::endl;
						outfile.close();
					}
					break;
				}
				case 'w':
				{
					//Empty the images & folders
					if (system("rm /home/pfc/sty/Documents/cameraCalib/build/*.txt") == -1);
					if (system("rm /home/pfc/sty/Documents/cameraCalib/build/charuco/*.png") == -1);
					if (system("rm /home/pfc/sty/Documents/cameraCalib/build/images/*.png") == -1);
					if (system("rm /home/pfc/sty/Documents/cameraCalib/build/depths/*.png") == -1);
					if (system("rm /home/pfc/sty/Documents/cameraCalib/build/images/left/*.png") == -1);
					if (system("rm /home/pfc/sty/Documents/cameraCalib/build/3Dreconstructed/matches/*") == -1);
					if (system("rm /home/pfc/sty/Documents/cameraCalib/build/3Dreconstructed/reconstruction_global/*") == -1);
				}
			}
			
			ProgressBar((float)(svo_position / (float)nb_frames), 30);
		}
		else if (returned_state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED)
		{
			print("SVO end has been reached. Looping back to 0\n");
			zed.setSVOPosition(0);
		}
		else
		{
			print("Grab ZED : ", returned_state);
			break;
		}
	}
		zed.close();
		if (system("mv ~/JustAFolder/* ~/smallTrash") == -1);
		return EXIT_SUCCESS;
}
