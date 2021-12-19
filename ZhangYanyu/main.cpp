#include "checkball.cpp"
#include "checkball.h"

int main()
{
    Check greenball;
    VideoCapture capture;
    capture.open(0);
                                                                                                                                
    if(!capture.isOpened()){
        cout<<"could not open camera...\n"<<endl;
    }
    namedWindow("result",WINDOW_AUTOSIZE);
    Mat frame;
    while(1){
        capture>>frame;
        bool i = greenball.findball(frame);
        cout<<i<<endl;
        waitKey(30);
    }
    

    return 0;
}