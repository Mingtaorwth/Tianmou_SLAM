#include "lyn_cam.h"
// #include <windows.h>
#include <iostream>
#include <fstream>
#include "tmc_proc.h"
#include <chrono>
#include <time.h>
#include <vector>
#include <cstdint>
#include <string>
#include <thread>
#include <string>
#include "isp.h"
#include <unistd.h>
#include "camera_param.hpp"
#include <future>
#include<iostream>
#include<memory>
#include<iterator>
#include<stdlib.h>
// #define USB_SDK
//  #pragma comment(lib, "SetupAPI.lib")
//  #pragma comment(lib, "user32.lib")
//  #pragma comment(lib, "legacy_stdio_definitions.lib")

// using namespace lynxicam;

struct CameraFrame{
    double timeStamp;
    int data_type;
    // 0 for rod 1 for cone 

    int rod_pvalue[CAMERA_ROD_PVALUE_LENGTH];
    int cone_pvalue[CAMERA_CONE_PVALUE_LENGTH];
    // cv::Mat raw = cv::Mat::zeros(CAMERA_CONE_H, CAMERA_CONE_W, CV_16SC1);
    // // 10位的bggr raw图，占用16位空间，返回每个感光原件的值，可以转换成rgb 这里不转换
    // cv::Mat timediff = cv::Mat::zeros(CAMERA_ROD_H, CAMERA_ROD_W, CV_8SC1);
    // cv::Mat spatioDiffl = cv::Mat::zeros(CAMERA_ROD_H, CAMERA_ROD_W, CV_8SC1);
    // cv::Mat spatioDiffr = cv::Mat::zeros(CAMERA_ROD_H, CAMERA_ROD_W, CV_8SC1);
};
// struct CameraImage{
//     cv::Mat raw = cv::Mat::zeros(CAMERA_CONE_H, CAMERA_CONE_W, CV_16SC1);
//     // 10位的bggr raw图，占用16位空间，返回每个感光原件的值，可以转换成rgb 这里不转换
//     cv::Mat timediff = cv::Mat::zeros(CAMERA_ROD_H, CAMERA_ROD_W, CV_8SC1);
//     cv::Mat spatioDiffl = cv::Mat::zeros(CAMERA_ROD_H, CAMERA_ROD_W, CV_8SC1);
//     cv::Mat spatioDiffr = cv::Mat::zeros(CAMERA_ROD_H, CAMERA_ROD_W, CV_8SC1);
// };
// struct cameraConeFrame{
//     std::chrono::high_resolution_clock::time_point timeStamp;
//     bool data_type;
//     unsigned int cone_count=0,rod_count=0;
//     // 0 for rod 1 for cone 
//     cv::Mat raw = cv::Mat::zeros(CAMERA_CONE_H, CAMERA_CONE_W, CV_16SC1);
//     cv::Mat timediff = cv::Mat::zeros(CAMERA_ROD_H, CAMERA_ROD_W, CV_8SC1);
//     cv::Mat spatioDiffl = cv::Mat::zeros(CAMERA_ROD_H, CAMERA_ROD_W, CV_8SC1);
//     cv::Mat spatioDiffr = cv::Mat::zeros(CAMERA_ROD_H, CAMERA_ROD_W, CV_8SC1);
// }
class DataController {
    public:
        // std::chrono::high_resolution_clock::time_point  last_cone_time,curr_cone_time,last_fetch_cone_time;
        // std::chrono::high_resolution_clock::time_point  last_rod_time,curr_rod_time,last_fetch_rod_time;
        double  last_cone_time,curr_cone_time,last_fetch_cone_time;
        double  last_rod_time,curr_rod_time,last_fetch_rod_time;
    
        std::chrono::duration<double,std::ratio<1,1>> duration_cone,duration_rod;
        

        DataController(int camera_id,char* sdk_path);
        DataController(int id);
        ~DataController();

        void startRead(std::chrono::high_resolution_clock::time_point global_init_time);
        void setFreqOutput();
        void unsetFreqOutput();

        void loadDataFromMat(CameraFrame &dataFrame);

        int rod_record_interval=1;


        cv::Mat getRaw();
        cv::Mat getTimeDiff();
        cv::Mat getSpatioDiffR();
        cv::Mat getSpatioDiffL();
        cv::Mat getRGB();
        cv::Mat getOpticalFlowHS();
        cv::Mat getKeyPoints();
        cv::Mat getOpticalFlowRecurrentMultipleScale();
        cv::Mat getReconstruction();
        cv::Mat getRGB_SD2();
        cv::Mat getRGB_SD1();
        cv::Mat getRGB_TD();
        cv::Mat getRGB_block();
        cv::Mat getSD2();
        cv::Mat getSD1();
        cv::Mat getTD();
        cv::Mat getHist();
        cv::Mat getIxy();
        void copyRodPValue(int des[CAMERA_ROD_PVALUE_LENGTH]);
        void copyConePValue(int des[CAMERA_CONE_PVALUE_LENGTH]);

        bool isFakeDevice();

    protected:
        int loopRead(std::chrono::high_resolution_clock::time_point global_init_time);
        void getDiff();
        lynFrame_t *cam_frames = new lynFrame_t();
        int rod_pvalue[CAMERA_FRAMES_BUFFER_LENGTH][CAMERA_ROD_PVALUE_LENGTH];
        int cone_pvalue[CAMERA_FRAMES_BUFFER_LENGTH][CAMERA_CONE_PVALUE_LENGTH];

        int camera_id;
        lynCameraHandle_t cameraHandle = nullptr;

        int countRod = 0, countCones = 0, conesFrameCount = 0, rodFrameCount = 0;

        const int h = CAMERA_ROD_H, w = CAMERA_ROD_W;
        const int ch = CAMERA_CONE_H, cw = CAMERA_CONE_W;
        // int DecodedBuffer[3*CAMERA_ROD_W*CAMERA_ROD_H];
        // int rodBuffer[CAMERA_ROD_BUFFER_LENGTH];
        int adc_bit_prec=8;
        int rod_interface = 3;
        
        int cone_effDataLength;
        int rod_effDataLength;
        std::ofstream conesfile;
        std::ofstream rodfile;

        // int cnt = 0;
        //   lynFrame_t cam_frames;
        uint rod_now_cnt = 0, rod_prev_cnt = 0;
        uint64_t time_prev = 0;

        int rod_cnt = 0;
        int cone_cnt = 0;

        bool if_freq_output=0;
        bool if_fake_device=false;

        bool enableRGBHist=false;
        bool enableFlipDisplay=false;
        bool enableStaticDenoise=true;
        bool denoiseInitFlag=false;
        bool enableFunction=false;
        bool enableOpticalFlow=true;
        bool enableReconstruction=true;
        bool enableKeyPoints=true;
        bool diffShowMode=1;
        bool enableHist=false;
        bool isBLCmode=false;
        bool BLCisReady=false;
        bool enableFullResolution = false;
        bool enableAWB = false;
        bool enableBLCDenoise = false;
        bool enableCorrDenoise=false;
        bool needSDCorrection = false;
        bool enableConeAE=false;

        const int threshSD = 100;
        const int threshTD = 5;
        const int binScaling = 1;
        const int scaling = 1;
        const int ofacum = 3;
        // const int kpacum = 3;
        // const int reccum = 1;
        int SDPeakValue=0;

        bool cone_iolock=0, rod_iolock=0;
        unsigned int cone_buffer_idx=0,rod_buffer_idx=0;
        const unsigned int frame_buffer_len=CAMERA_FRAMES_BUFFER_LENGTH;
        cv::Mat raw = cv::Mat::zeros(ch, cw*2, CV_16SC1);
        cv::Mat timediff=cv::Mat::zeros(h, w, CV_8SC1);
        cv::Mat spatioDiffl=cv::Mat::zeros(h, w, CV_8SC1);
        cv::Mat spatioDiffr=cv::Mat::zeros(h, w, CV_8SC1);
        cv::Mat TD = cv::Mat::zeros(h, w*2, CV_8SC1);
        cv::Mat SD1 = cv::Mat::zeros(h, w*2, CV_8SC1);
        cv::Mat SD2 = cv::Mat::zeros(h, w*2, CV_8SC1);
        cv::Mat RGB_TD = cv::Mat::zeros(h, w*2, CV_8UC3);
        cv::Mat RGB_SD1 = cv::Mat::zeros(h, w*2, CV_8UC3);
        cv::Mat RGB_SD2 = cv::Mat::zeros(h, w*2, CV_8UC3);

        // cv::Mat SD1Accum;
        // cv::Mat SD2Accum;
        cv::Mat opticalFlow3C = cv::Mat::zeros(h, w*2, CV_32FC3); ;
        cv::Mat SD_recon=cv::Mat::zeros(h, w*2, CV_8UC1);
        cv::Mat TDdenoiserAccum;
        cv::Mat SD1denoiserAccum;
        cv::Mat SD2denoiserAccum;
        std::vector<cv::Mat> TDdenoiser_FIFO;
        std::vector<cv::Mat> SD1denoiser_FIFO ;
        std::vector<cv::Mat> SD2denoiser_FIFO ;
        cv::Mat oldFrameTD;
        cv::Mat oldFrameSD1;
        cv::Mat oldFrameSD2;
        cv::Mat SDTDhist;
        cv::Mat TDAccum=cv::Mat::zeros(h, w, CV_32FC1);
        cv::Mat sdlAccum=cv::Mat::zeros(h, w, CV_32FC1);
        cv::Mat sdrAccum=cv::Mat::zeros(h, w, CV_32FC1);
        cv::Mat sdlAccum_ave=cv::Mat::zeros(h, w, CV_32FC1);
        cv::Mat sdrAccum_ave=cv::Mat::zeros(h, w, CV_32FC1);
        cv::Mat Ix= cv::Mat::zeros(h, w, CV_32FC1);
        cv::Mat Iy= cv::Mat::zeros(h, w, CV_32FC1);
        cv::Mat kpResult;
        void accuOpticalFlowRecurrentMultipleScale();
        unsigned int last_read_TimeDiff_id=0,last_read_SpatioDiffL_id=0,last_read_SpatioDiffR_id=0,last_read_Diff_id=0;
        unsigned int last_read_raw_id=0,last_read_rgb_id=0; 
        unsigned int last_read_TD_id=0,last_read_SD1_id=0,last_read_SD2_id=0;
        unsigned int last_read_RGB_TD_id=0,last_read_RGB_SD1_id=0,last_read_RGB_SD2_id=0;
        unsigned int last_read_Hist_id=0;
        unsigned int lasr_read_reconstruction_id=0;
        cv::Mat resized;
        cv::Mat RGBhist;
        cv::Mat runtimeConeBLC;
        int BLCFRAME;

        float Rcor =1.0;
        float GCor =0.8;
        float Bcor =1.1;

};