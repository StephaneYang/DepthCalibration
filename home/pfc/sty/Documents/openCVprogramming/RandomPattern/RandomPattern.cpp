#include <stdlib.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ccalib/randpattern.hpp>

#include <iostream>

using namespace std;
using namespace cv;

int main( int argc, char** argv )
{
    if(system("rm *.png"));

    int width = 595, height = 842, i = 0;

    if ((argc == 2)||(argc == 3))
        width = atoi(argv[1]);

    if (argc == 3)
        height = atoi(argv[2]);
        
    for (i=0; i<10; i++) {
	    randpattern::RandomPatternGenerator generator(width, height);
	    generator.generatePattern();
	    Mat pattern = generator.getPattern();

	    imshow("Pattern "+to_string(i), pattern);
	    imwrite("random_pattern_"+to_string(i)+".png", pattern);
	    waitKey(0);
    }
    
    
    return 0;
}
