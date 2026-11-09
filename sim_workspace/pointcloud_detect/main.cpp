#include "src/road_detect.h"
#include "src/object_detect.h"

void signalHandler(int signum) {
    cout << "Interrupt signal (" << signum << ") received.\n";
    exit(-1);
}

int main(int argc ,char **argv) {
    signal(SIGINT, signalHandler);
    signal(SIGSEGV, signalHandler);
    signal(SIGTERM, signalHandler);

    ros::init(argc, argv,"road_detect");
    std::string path = "";
    int mode = atoi(argv[1]);
    if (mode == MODE_POINTCLOUD_MAP_SEG) {
        path = argv[2];
    }
    PointCloudRoadDetect roaddetect(mode, path);
    
    cout<<" =========start loop=============\n"; 
    ros::spin();
    return 0;
}