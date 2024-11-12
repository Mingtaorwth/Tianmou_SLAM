#include "camera_data_server.hpp"
#include <filesystem>
class DualCamera{
    public:
        DataController *cameraL = nullptr;
        DataController *cameraR = nullptr;
        DualCamera();
        ~DualCamera();

        void DataRecorder(double start_time, double end_time, int max_length, char* log_folder_path);
        void DataReplay(char* log_folder_path);
        void DataListener();
        void SingleDataListener();
        void DataLoader(char* log_folder_path);
        int n_device=0;
    protected:
        void DataReplayLoop(char* log_folder_path);
};