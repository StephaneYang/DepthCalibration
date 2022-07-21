#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include "matplotlibcpp.h"
#include <cmath>

using namespace std;
namespace plt = matplotlibcpp;

int main(int argc, char **argv)
{
	std::vector<double> j(2);
	j.at(0) = 0;
	j.at(1) = 1000;
	
	plt::figure(1);
	// Set the size of output image to 2000x2000 pixels
	plt::figure_size(2000, 2000);
	// plot a diagonal for comparison
	plt::plot(j, j, "tab:gray");
	for (int id = 0; id < 24; id++)
	{
		// Prepare data.
		int k = 0;
		std::vector<double> x(1000), y(1000);
		fstream newfile;
		newfile.open("/home/pfc/sty/Documents/cameraCalib/build/depthComparisonId"+to_string(id)+".txt",ios::in); //open a file to perform read operation using file object
		if (newfile.is_open())
		{//checking whether the file is open
			string tp;
			while(getline(newfile, tp)) //read data from file object and put it into string.
			{
				cout << tp << "\n"; //print the data of the string

				if(tp.length()>110)
				{
					string token1 = tp.substr(tp.find(" mm depth(calculated)")-10, 10);
					cout << token1 << endl;
					y.at(k) = stod(token1);
					cout << y.at(k) << endl;
					
					string token2 = tp.substr(tp.find(" mm depth(ZED)")-10, 10);
					cout << token2 << endl;
					x.at(k) = stod(token2);
					cout << x.at(k) << endl;
					
					k++;
				}
				else
				{
					cout << "no depth found\n";
				}
			}
		}
		newfile.close(); //close the file object.
		
		std::vector<double> z(k), w(k);
		double a, b;
		for(int i=0; i<k; i++)
		{
				z.at(i) = x.at(i);
				w.at(i) = y.at(i);
		}
		// Plot line from given x and y data. Color is selected automatically.
		plt::plot(z, w, {{"label", "Id "+to_string(id)}});
	}
	// Add graph title
	plt::title("Depths comparison");
	// Enable legend.
	plt::legend();
	// axis limit 0 to 1000 by default
	if (argc == 3)
	{
		plt::xlim(stoi(argv[1]), stoi(argv[2]));
		plt::ylim(stoi(argv[1]), stoi(argv[2]));
	}
	else
	{
		plt::xlim(0, 1000);
		plt::ylim(0, 1000);
	}
	// axis label
	plt::xlabel("Depth from ZED camera");
	plt::ylabel("Depth from calculation");
	// Save the image (file format is determined by the extension)
	plt::save("./DepthPlot.png");
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	plt::figure(2);
	// Set the size of output image to 2000x2000 pixels
	plt::figure_size(2000, 2000);
	// plot a diagonal for comparison
	plt::plot(j, j, "tab:gray");
	for (int id = 0; id < 24; id++)
	{
		// Prepare data.
		int k = 0;
		std::vector<double> x(1000), y(1000);
		fstream newfile;
		newfile.open("/home/pfc/sty/Documents/cameraCalib/build/depthComparisonId"+to_string(id)+".txt",ios::in); //open a file to perform read operation using file object
		if (newfile.is_open())
		{//checking whether the file is open
			string tp;
			while(getline(newfile, tp)) //read data from file object and put it into string.
			{
				cout << tp << "\n"; //print the data of the string

				if(tp.length()>110)
				{
					string token1 = tp.substr(tp.find(" mm depth(calculated)")-10, 10);
					cout << token1 << endl;
					y.at(k) = stod(token1);
					cout << y.at(k) << endl;
					
					string token2 = tp.substr(tp.find(" mm depth(ZED)")-10, 10);
					cout << token2 << endl;
					x.at(k) = stod(token2);
					cout << x.at(k) << endl;
					
					k++;
				}
				else
				{
					cout << "no depth found\n";
				}
			}
		}
		newfile.close(); //close the file object.
		
		std::vector<double> z(k), w(k);
		double a, b;
		for(int i=0; i<k; i++)
		{
				z.at(i) = x.at(i);
				w.at(i) = y.at(i);
		}
		// Plot line from given x and y data. Color is selected automatically.
		plt::scatter(z, w, {{"label", "Id "+to_string(id)}});
	}
	// Add graph title
	plt::title("Depths comparison");
	// Enable legend.
	plt::legend();
	// axis limit 0 to 1000 by default
	if (argc == 3)
	{
		plt::xlim(stoi(argv[1]), stoi(argv[2]));
		plt::ylim(stoi(argv[1]), stoi(argv[2]));
	}
	else
	{
		plt::xlim(0, 1000);
		plt::ylim(0, 1000);
	}
	// axis label
	plt::xlabel("Depth from ZED camera");
	plt::ylabel("Depth from calculation");
	// Save the image (file format is determined by the extension)
	plt::save("./DepthScatter.png");
}
