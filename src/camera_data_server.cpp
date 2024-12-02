#include "camera_data_server.hpp"
// #define USB_SDK
//  #pragma comment(lib, "SetupAPI.lib")
//  #pragma comment(lib, "user32.lib")
//  #pragma comment(lib, "legacy_stdio_definitions.lib")

// using namespace lynxicam;

DataController::DataController(int id)
{
    char *TIANMOUC_SDK_DIR; 
    if((TIANMOUC_SDK_DIR = getenv("TIANMOUC_SDK_DIR"))) 
    new (this) DataController(id, TIANMOUC_SDK_DIR);
}
DataController::DataController(int id, char *sdk_path)
{
    // for (int i=1;i<frame_buffer_len;i++){
    //     raw[i]=raw[0].clone();
    //     timediff[i]=timediff[0].clone();
    //     spatioDiffl[i]=spatioDiffl[0].clone();
    //     spatioDiffr[i]=spatioDiffr[0].clone();
    // }
    camera_id = id;
    if (camera_id < 0)
    {
        if_fake_device=true;
        rod_iolock=1;
        cone_iolock=1;
        return;
        // we need to do nothing, the camera_id<0 are used to represent fake device (like when we want to connect to the data recorder )
    }
    cameraHandle = nullptr;
    lynOpenCamera(&cameraHandle);
    lynCameraInit(cameraHandle, camera_id);
    printf("init %d Done", camera_id);
    lynCameraConfigFPGAMode(cameraHandle, adc_bit_prec, rod_interface);

    sleep(1);
    // Here type your iic configuration path!
    std::string str_sdk_path(sdk_path);
    std::string cfg_file_path = str_sdk_path + "tmc_cfg_files/para/750fps_8bit.csv";
    if (rod_interface == 0)
    {
        cfg_file_path = str_sdk_path + "tmc_cfg_files/lvds/1500fps_8bit.csv";
    }
    else
    {
        cfg_file_path = str_sdk_path + "tmc_cfg_files/para/750fps_8bit.csv";
        //"./C1_8bit_PARA_fixed_HCG_th1.csv";
    }
    if (!(access(cfg_file_path.c_str(), F_OK) == 0))
    {
        fprintf(stderr, "error: file %s not exists\n", cfg_file_path.c_str());
    }
    lynCameraConfigSensorFull(cameraHandle, cfg_file_path.c_str());
    // configure camera again..
    lynCameraConfigFPGAMode(cameraHandle, adc_bit_prec, rod_interface);
    lynStartRecvData(cameraHandle); // 打开数据接收、开了多个线程

}

void DataController::startRead(std::chrono::high_resolution_clock::time_point global_init_time)
{
    // std::future<int> read_cam = std::async(std::launch::async, &DataController::loopRead, this);
    std::thread read_cam(&DataController::loopRead, this, global_init_time);
    // read_cam.join();
    

    cpu_set_t cpuset;
 
    CPU_ZERO(&cpuset);
 
    CPU_SET(camera_id+1,&cpuset);
 
    int rc =pthread_setaffinity_np(read_cam.native_handle(),sizeof(cpu_set_t), &cpuset);

    read_cam.detach();
 
}
int DataController::loopRead(std::chrono::high_resolution_clock::time_point global_init_time)
{

    printf("Camera \033[1;31m%d\033[0m , handle \033[1;31m%x\033[0m, begin read\n", camera_id, &cameraHandle);
    // printf("Camera \033[1;31m%d\033[0m , handle \033[1;31m%x\033[0m, begin read\n", camera_id, &cameraHandle);
    // if(global_init_time){
    //     global_init_time = std::chrono::high_resolution_clock::now();
    // }else{
    //     // last_rod_time = global_init_time;
    // }
    curr_cone_time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(std::chrono::high_resolution_clock::now() - global_init_time).count();
    last_cone_time = curr_cone_time;

    curr_rod_time = curr_cone_time;
    last_rod_time = curr_rod_time;

    // cv::Mat test=cv::Mat::zeros(ch, ch, CV_16SC1);
    while (1)
    {
        // uint32_t* p = (uint32_t*)imagebuffer;
        if (lynGetFrame(cam_frames, cameraHandle))
        {
            if (cam_frames->dataType == LYN_CONE)
            {
                
                cone_cnt++;
                cone_effDataLength = cam_frames->length / 4;
                // cone_effDataLength = 16 + 320 * 160;
                cone_iolock = 1;
                memcpy(cone_pvalue[(cone_buffer_idx+1)%frame_buffer_len], (int *)cam_frames->data, sizeof(int) * cone_effDataLength);
                lynPutFrame(cam_frames, cameraHandle);
                // tianmouc::process::cone_reader(cone_pvalue, raw, cw,ch);
                cone_iolock = 0;

                last_cone_time = curr_cone_time;
                curr_cone_time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(std::chrono::high_resolution_clock::now() - global_init_time).count();

                cone_buffer_idx=(cone_buffer_idx+1)%frame_buffer_len;
                // 写入完成后再加
            }
            else if (cam_frames->dataType == LYN_ROD)
            {
                //  printf("now read rod %d\n", rod_cnt);
                
                rod_effDataLength = cam_frames->length / 4;
                int *data_print = (int *)cam_frames->data;
                // printf("length %d, data0 %x\n", length, data_print[0]);
                if (rod_effDataLength > 147168 / 4)
                {
                    //    int overflow_pos = length - 147168;
                    // for(auto i = 0; i< rod_effDataLength; i++){
                    //    printf("%x\n ", data_print[i]);
                    //}
                    printf("camera %d Overflow! rod_effDataLength %d, data0 %x\n", camera_id, rod_effDataLength, data_print[0]);
                }
                rod_iolock = 1;
                memcpy(rod_pvalue[(rod_buffer_idx+1)%frame_buffer_len], (int *)cam_frames->data, sizeof(int) * rod_effDataLength);
                lynPutFrame(cam_frames, cameraHandle);
                // get cv format matrix

                // int* rod_pvalue = (int*)cam_frames.data;
                rod_now_cnt = tianmouc::process::get_rod_counter(rod_pvalue[(rod_buffer_idx+1)%frame_buffer_len]);
                rod_cnt++;
                // if (_qDEBUG_USB_READ) {
                int64_t delta_cnt = (int64_t)rod_now_cnt - (int64_t)rod_prev_cnt;
                if (delta_cnt != 1)
                {
                    uint64_t timestamp = tianmouc::process::get_rod_timestamp(rod_pvalue[(rod_buffer_idx+1)%frame_buffer_len]);
                    std::cout << "Rod May loss this cnt " << rod_now_cnt << ", previous cnt: " << rod_prev_cnt << ", delta: " << delta_cnt
                              << ", this timestamp " << timestamp << ", Delta Time " << 10 * ((int64_t)timestamp - (int64_t)time_prev) << "us" << std::endl;
                    time_prev = timestamp;
                }
                rod_prev_cnt = rod_now_cnt;

                rod_iolock = 0;
                rod_buffer_idx=(rod_buffer_idx+1)%frame_buffer_len;

                rod_cnt = 0;
                last_rod_time = curr_rod_time;
                curr_rod_time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(std::chrono::high_resolution_clock::now() - global_init_time).count();;
            }
            if (if_freq_output == true)
            {
                std::cout << "camera id " << camera_id << " freq rod  " << 1 / (curr_rod_time - last_rod_time) << "   cone  " << 1 / (curr_cone_time - last_cone_time) << "\033[A\r" << std::endl;
            }
        }
    }
    return true;
}
cv::Mat DataController::getRaw()
// OpenCV 4.5.2 cv::Mat的赋值操作符重载源码，添加了说明注释
// modules/core/src/matrix.cpp
// 直接赋值并不会增加变量空间
{
    // while(cone_iolock==1){

    // }
    // unsigned int curr_idx=cone_buffer_idx;
    tianmouc::process::cone_reader(cone_pvalue[(cone_buffer_idx)%frame_buffer_len],
                                    raw,
                                    cw,
                                    ch);
    last_read_raw_id++;
    return raw;
}

void DataController::getDiff(){

    int code = tianmouc::process::rod_decoder((int *)rod_pvalue[rod_buffer_idx%frame_buffer_len], timediff,
                                                spatioDiffl, spatioDiffr, w, h, rod_effDataLength);
    double threshold = 32.0; // Replace 100.0 with your desired threshold value
    cv::Mat thresholdMask1 = (cv::abs(spatioDiffl) > threshold);
    cv::Mat thresholdMask2 = (cv::abs(spatioDiffr) > threshold);
    int count1 = cv::countNonZero(thresholdMask1);
    int count2 = cv::countNonZero(thresholdMask2);
    //std::cout << "Number of elements exceeding the threshold: " << count1+count2 << std::endl;
    this-> SDPeakValue += (count1+count2);
    // this-> SDPeakvalueAccumulationCount+= 1;

    // if(SDPeakvalueAccumulationCount%4 == 0){
    //     QMetaObject::invokeMethod(this->Window,"getSDPeakValue",Q_ARG(int,count1+count2),Q_ARG(int,globalCameraIdx));
    //     this-> SDPeakValue = 0;
    // }

    if(this->enableFlipDisplay){
        cv::flip(timediff,timediff,1);
        cv::flip(spatioDiffl,spatioDiffl,1);
        cv::flip(spatioDiffr,spatioDiffr,1);
    }
    if(enableStaticDenoise){
        if(!denoiseInitFlag){
            denoiseInitFlag = true;
            TDdenoiserAccum = cv::Mat::zeros(this->h, this->w, CV_32F);
            SD1denoiserAccum = cv::Mat::zeros(this->h, this->w, CV_32F);
            SD2denoiserAccum = cv::Mat::zeros(this->h, this->w, CV_32F);
            //Taoyi changed to image fifo (size of 2) for moving average

        }else{
            timediff.convertTo(timediff,CV_32F);
            spatioDiffl.convertTo(spatioDiffl,CV_32F);
            spatioDiffr.convertTo(spatioDiffr,CV_32F);
            tianmouc::isp::denoiseAcum(timediff,TDdenoiserAccum,last_read_Diff_id);
            // tianmouc::isp::denoiseAcum(spatioDiffl,SD1denoiserAccum,last_read_Diff_id);
            // tianmouc::isp::denoiseAcum(spatioDiffr,SD2denoiserAccum,last_read_Diff_id);
            tianmouc::isp::denoise_moving_ave(timediff, TDdenoiser_FIFO, last_read_Diff_id, 128);
            // tianmouc::isp::denoise_moving_ave(spatioDiffl, SD1denoiser_FIFO, last_read_Diff_id,  128);
            // tianmouc::isp::denoise_moving_ave(spatioDiffr, SD2denoiser_FIFO, last_read_Diff_id,  128);
            timediff.convertTo(timediff,CV_8SC1);
            spatioDiffl.convertTo(spatioDiffl,CV_8SC1);
            spatioDiffr.convertTo(spatioDiffr,CV_8SC1);
        }
    }

    // if(enableOpticalFlow | enableReconstruction | enableKeyPoints){
    //     // uchar  cpdata[w*h*3];
    //     memcpy(DecodedBuffer, timediff.data, w*h * sizeof(uchar));
    //     memcpy(DecodedBuffer +  w*h, spatioDiffl.data, w*h * sizeof(uchar));
    //     memcpy(DecodedBuffer + 2*w*h, spatioDiffr.data, w*h * sizeof(uchar));
    //     // DecodedBuffer[globalCameraIdx]->writeRodData(cpdata,w*h*3,timestamp);
    // }
    // if (enableOpticalFlow){
    //     accuOpticalFlowRecurrentMultipleScale();
    // }
    // if(enableRodAE){
    //     uchar  cpdata2[w*h*3];
    //     memcpy(cpdata2, timediff.data, w*h * sizeof(uchar));
    //     memcpy(cpdata2 + w*h, spatioDiffl.data, w*h * sizeof(uchar));
    //     memcpy(cpdata2 + 2*w*h, spatioDiffr.data, w*h * sizeof(uchar));
    //     AEBuffer[globalCameraIdx]->writeRodData(cpdata2,w*h*3,timestamp);
    // }
    cv::resize(timediff, TD, cv::Size(w*2, h), 0, 0, cv::INTER_NEAREST);
    // cv::resize(spatioDiffl, SD1, cv::Size(w*2, h), 0, 0, cv::INTER_NEAREST);
    // cv::resize(spatioDiffr, SD2, cv::Size(w*2, h), 0, 0, cv::INTER_NEAREST);
    // if (diffShowMode==1){
    //     RGB_TD = cv::Mat::zeros(h, w*2, CV_8UC3);
    //     RGB_SD1 = cv::Mat::zeros(h, w*2, CV_8UC3);
    //     RGB_SD2 = cv::Mat::zeros(h, w*2, CV_8UC3);
    // }

    // if(enableCorrDenoise){
    //     if(last_read_Diff_id>1){
    //         tianmouc::isp::denoiseCorr(RGB_SD1,oldFrameSD1);
    //         tianmouc::isp::denoiseCorr(RGB_SD2,oldFrameSD2);
    //         tianmouc::isp::denoiseCorr(RGB_TD,oldFrameTD);
    //     }else{
    //         oldFrameTD = RGB_TD.clone();
    //         oldFrameSD1 = RGB_SD1.clone();
    //         oldFrameSD2 = RGB_SD2.clone();
    //     }
    //     oldFrameTD = RGB_TD.clone();
    //     oldFrameSD1 = RGB_SD1.clone();
    //     oldFrameSD2 = RGB_SD2.clone();
    // }

    // tianmouc::isp::convertGraytoRGB_Fast(TD, RGB_TD, scaling, threshTD, diffShowMode);
    // if(needSDCorrection){
    //     tianmouc::isp::convertGraytoRGB_SD_exchange(SD1,SD2,RGB_SD1, RGB_SD2, scaling, threshSD, diffShowMode);
    // }else{
    //     tianmouc::isp::convertGraytoRGB_Fast(SD1, RGB_SD1, scaling, threshSD, diffShowMode);
    //     tianmouc::isp::convertGraytoRGB_Fast(SD2, RGB_SD2, scaling, threshSD, diffShowMode);
    // }
    // if(this->isRunning){
    //     //fig1
    //     // if(enableHist){
    //     //     QMetaObject::invokeMethod(this->Window,"setGraphPanel",
    //     //                          Q_ARG(QImage,QImage((const uchar*)SDTDhist.data,SDTDhist.cols,SDTDhist.rows,SDTDhist.step,
    //     //                                       QImage::Format_RGB888)),Q_ARG(int,6),Q_ARG(int,globalCameraIdx));
    //     // }
    //     //fig2
    //     QMetaObject::invokeMethod(this->Window,"setGraphPanel",
    //                               Q_ARG(QImage,QImage((const uchar*)RGB_TD.data,RGB_TD.cols,RGB_TD.rows,RGB_TD.step,
    //                                            QImage::Format_RGB888)),Q_ARG(int,1),Q_ARG(int,globalCameraIdx));
    //     //fig3
    //     if(!enableFunction){
    //         QMetaObject::invokeMethod(this->Window,"setGraphPanel",
    //                                   Q_ARG(QImage,QImage((const uchar*)RGB_SD1.data,RGB_SD1.cols,RGB_SD1.rows,RGB_SD1.step,
    //                                                QImage::Format_RGB888)),Q_ARG(int,2),Q_ARG(int,globalCameraIdx));
    //     }
    //     //fig4
    //     QMetaObject::invokeMethod(this->Window,"setGraphPanel",
    //                               Q_ARG(QImage,QImage((const uchar*)RGB_SD2.data,RGB_SD2.cols,RGB_SD2.rows,RGB_SD2.step,
    //                                            QImage::Format_RGB888)),Q_ARG(int,3),Q_ARG(int,globalCameraIdx));

    // }

    last_read_Diff_id++;
    return ;
}

cv::Mat DataController::getHist(){
    if(last_read_Hist_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        last_read_Hist_id=last_read_Diff_id;
    }
    SDTDhist = tianmouc::isp::histogramSDTD(spatioDiffl,spatioDiffr,timediff,2<<7,binScaling);
    return SDTDhist;
}
cv::Mat DataController::getTimeDiff(){
    // while(rod_iolock==1){
        
    // }
    if(last_read_TimeDiff_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        last_read_TimeDiff_id=last_read_Diff_id;
    }
    // cv::Mat temp=timediff;
    // temp.convertTo(temp, CV_16SC1);
    // temp = temp + 128;
    // temp.convertTo(temp, CV_8UC1);
    
    return timediff;
}

cv::Mat DataController::getSpatioDiffR(){
    // while(rod_iolock==1){
        
    // }
    if(last_read_SpatioDiffR_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        last_read_SpatioDiffR_id=last_read_Diff_id;
    }

    // cv::Mat temp=spatioDiffr;
    // temp.convertTo(temp, CV_16SC1);
    // temp = temp + 128;
    // temp.convertTo(temp, CV_8UC1);
    
    return spatioDiffr;
}
cv::Mat DataController::getSpatioDiffL(){
    // while(rod_iolock==1){
        
    // }
    if(last_read_SpatioDiffL_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        last_read_SpatioDiffL_id=last_read_Diff_id;
    }
    // cv::Mat temp=spatioDsiffl;

    // temp.convertTo(temp, CV_16SC1);
    // temp = temp + 128;
    // temp.convertTo(temp, CV_8UC1);
    
    return spatioDiffl;
}
cv::Mat DataController::getTD(){
    if(last_read_TD_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        last_read_TD_id=last_read_Diff_id;
    }
    
    cv::Scalar meanVal = cv::mean(TD);

    std::cout << "Mean Value: " << meanVal[0] << std::endl;

    double lowThreshold1 = -9;
    double highThreshold1 = -5;

    double lowThreshold2 = 5;
    double highThreshold2 = 9;


    // 创建两个掩码，分别用于 -20 到 -5 和 5 到 20 的像素
    cv::Mat mask1, mask2;
    cv::inRange(TD, lowThreshold1, highThreshold1, mask1);
    cv::inRange(TD, lowThreshold2, highThreshold2, mask2);

    // 合并两个掩码
    cv::Mat combinedMask;
    cv::bitwise_or(mask1, mask2, combinedMask);


    cv::Mat filtertd;
    TD.copyTo(filtertd, combinedMask);

    cv::Mat td;
    cv::normalize(filtertd, td, 0, 220, cv::NORM_MINMAX, CV_8UC3);
    td = td + 50;
    // filtertd.convertTo(td, CV_8UC3);
    
    return td;
}

cv::Mat DataController::getSD1(){
    if(last_read_SD1_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        last_read_SD1_id=last_read_Diff_id;
    }
    return SD1;
}
cv::Mat DataController::getSD2(){
    if(last_read_SD2_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        last_read_SD2_id=last_read_Diff_id;
    }
    return SD2;
}
cv::Mat DataController::getRGB_TD(){
    if(last_read_RGB_TD_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        last_read_RGB_TD_id=last_read_Diff_id;
    }
    return RGB_TD;
}
cv::Mat DataController::getRGB_SD1(){
    if(last_read_RGB_SD1_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        last_read_RGB_SD1_id=last_read_Diff_id;
    }
    return RGB_SD1;
}
cv::Mat DataController::getRGB_SD2(){
    if(last_read_RGB_SD2_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        last_read_RGB_SD2_id=last_read_Diff_id;
    }
    return RGB_SD2;
}
cv::Mat DataController::getReconstruction(){
    if(lasr_read_reconstruction_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        lasr_read_reconstruction_id=last_read_Diff_id;
    }
            // cv::resize(spatioDiffl, SD1, cv::Size(w*2, h), 0, 0, cv::INTER_NEAREST);
            // cv::resize(spatioDiffr, SD2, cv::Size(w*2, h), 0, 0, cv::INTER_NEAREST);
            // SD_recon = cv::Mat::zeros(h, w*2, CV_8UC1);
            tianmouc::isp::spatioReconstruct(SD1,SD2,SD_recon);
            // QMetaObject::invokeMethod(this->mainWindow,"setGraphPanel",
            //                           Q_ARG(QImage,QImage((const uchar*)SD_recon.data,SD_recon.cols,SD_recon.rows,
            //                           QImage::Format_Indexed8)),Q_ARG(int,2),Q_ARG(int,globalCameraIdx));
    return SD_recon;
}
cv::Mat DataController::getOpticalFlowRecurrentMultipleScale(){
    if(lasr_read_reconstruction_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        lasr_read_reconstruction_id=last_read_Diff_id;
    }
    tianmouc::isp::SDlSDR_2_SDxSDy(sdlAccum,sdrAccum,Ix,Iy);
    tianmouc::isp::opticalFlowRecurrentMultipleScale(sdlAccum,sdrAccum,TDAccum,opticalFlow3C,2);
    cv::resize(opticalFlow3C, opticalFlow3C, cv::Size(w*2, h), 0, 0, cv::INTER_NEAREST);
    opticalFlow3C.convertTo(opticalFlow3C,CV_8UC3, 255.0f);
    // QMetaObject::invokeMethod(this->mainWindow,"setGraphPanel",
    //                             Q_ARG(QImage,QImage((const uchar*)opticalFlow3C.data,opticalFlow3C.cols,opticalFlow3C.rows,
    //                                         QImage::Format_RGB888)),Q_ARG(int,2),Q_ARG(int,globalCameraIdx));
    return opticalFlow3C;
}
void DataController::accuOpticalFlowRecurrentMultipleScale(){
    TDAccum=timediff;
    sdlAccum+=spatioDiffl;
    sdlAccum/=2;
    sdrAccum+=spatioDiffr;
    sdrAccum/=2;
}
cv::Mat DataController::getKeyPoints(){
    if(lasr_read_reconstruction_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        lasr_read_reconstruction_id=last_read_Diff_id;
    }
    // if(last_read_Diff_id%kpacum==0){
        tianmouc::isp::SDlSDR_2_SDxSDy(spatioDiffl,spatioDiffr,Ix,Iy);
        kpResult = cv::Mat::zeros(h, w, CV_8SC1);
        tianmouc::isp::FAST_kp(Ix,Iy,kpResult,0.02, 0.02);
        cv::resize(kpResult, kpResult, cv::Size(w*2, h), 0, 0, cv::INTER_NEAREST);
        //qDebug()<<"FAST_kp Time  :"<<diff/1000.0<<"(ms)";
        // QMetaObject::invokeMethod(this->mainWindow,"setGraphPanel",
        //                             Q_ARG(QImage,QImage((const uchar*)kpResult.data,kpResult.cols,kpResult.rows,
        //                                         QImage::Format_Indexed8)),Q_ARG(int,2),Q_ARG(int,globalCameraIdx));
    // }
    return kpResult;
}

void conv_and_threshold_inplace(cv::Mat& input_image, int kernel_size, float threshold) {
    // 创建一个值全为1的卷积核
    cv::Mat conv_kernel = cv::Mat::ones(kernel_size, kernel_size, CV_32F);

    // 创建存储卷积结果的矩阵
    cv::Mat conv_output;

    // 应用卷积操作, 使用常量边界类型（BORDER_CONSTANT）
    cv::filter2D(input_image, conv_output, -1, conv_kernel, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);

    // 生成阈值掩码
    cv::Mat mask = conv_output > threshold;

    // 在原图像上应用掩码，掩码为 True 时保留原值，否则设置为 0
    input_image.setTo(0, ~mask); // 将掩码为 False 的部分设为 0
}

void conv_and_threshold_inplace_2(cv::Mat& input_image, int kernel_size, float threshold) {
    // 取图像的绝对值，确保数据类型为浮点数
    cv::Mat abs_image;
    cv::absdiff(input_image, cv::Scalar(0), abs_image); // 计算绝对值

    // 创建一个卷积核，全为 1，但中间元素为 0
    cv::Mat conv_kernel = cv::Mat::ones(kernel_size, kernel_size, CV_32F);
    conv_kernel.at<float>(kernel_size / 2, kernel_size / 2) = 0;  // 设置中心元素为 0

    // 创建存储卷积结果的矩阵
    cv::Mat conv_output;

    // 应用卷积操作，使用常量边界类型（BORDER_CONSTANT）
    cv::filter2D(abs_image, conv_output, -1, conv_kernel, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);

    // 生成阈值掩码
    cv::Mat mask = conv_output > threshold;

    // 在原图像上应用掩码，掩码为 True 时保留原值，否则设置为 0
    input_image.setTo(0, ~mask); // 将掩码为 False 的部分设为 0
}

void splitImage(const cv::Mat& input, cv::Mat& evenRows, cv::Mat& oddRows) {
    // 创建子矩阵，但不复制数据
    evenRows =cv::Mat((input.rows + 1) / 2, input.cols, input.type(), input.data);
    oddRows = cv::Mat(input.rows / 2, input.cols, input.type(), input.data + input.step);

    // 为了兼容不同大小的行数（奇数行图像），将 step 设置为 2 行的跨度
    evenRows = cv::Mat((input.rows + 1) / 2, input.cols, input.type(), input.data, input.step * 2);
    oddRows = cv::Mat(input.rows / 2, input.cols, input.type(), input.data + input.step, input.step * 2);
}

cv::Mat DataController::getIxy(){
     if(lasr_read_reconstruction_id==last_read_Diff_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getDiff();
        lasr_read_reconstruction_id=last_read_Diff_id;

    // spatioDiffl.convertTo(spatioDiffl, CV_16SC1);
    // spatioDiffl = spatioDiffl + 128;
    // spatioDiffl.convertTo(spatioDiffl, CV_8UC1);

    // spatioDiffr.convertTo(spatioDiffr, CV_16SC1);
    // spatioDiffr = spatioDiffr + 128;
    // spatioDiffr.convertTo(spatioDiffr, CV_8UC1);
    }

    // cv::Mat sdul, sdll, sdur, sdlr, SD_1, LSD_reconstructed, RSD_reconstructed, spatioDiffll, spatioDiffrr;
    // spatioDiffl.convertTo(spatioDiffll, CV_32F);
    // spatioDiffr.convertTo(spatioDiffrr, CV_32F);

    // splitImage(spatioDiffll, sdul, sdll);
    // splitImage(spatioDiffrr, sdur, sdlr);
    
    // cv::Mat TD_1 = (sdul - sdlr) / 2.0;
    // conv_and_threshold_inplace(TD_1, 3, 2);
    // conv_and_threshold_inplace_2(TD_1, 3, 6);
    
    // // 生成阈值掩码 TD_10 = np.abs(TD_1) >= thr_1
    // cv::Mat TD_10;
    // cv::absdiff(TD_1, cv::Scalar(0), TD_10); // 取绝对值
    // TD_10 = TD_10 >= 6;  // 使用阈值生成布尔掩码

    // cv::Mat TD_2 = (sdll - sdur) / 2.0;
    // conv_and_threshold_inplace(TD_2, 3, 2);
    // conv_and_threshold_inplace_2(TD_2, 3, 6);


    // cv::Mat TD_20;
    // cv::absdiff(TD_2, cv::Scalar(0), TD_20); // 取绝对值
    // TD_20 = TD_20 >= 6;  // 使用阈值生成布尔掩码

    // // 创建 LSD_reconstructed 和 RSD_reconstructed
    // std::vector<cv::Mat> lsd_channels = {TD_10, TD_20};  // 将TD_10和TD_20合并为一个向量
    // std::vector<cv::Mat> rsd_channels = {TD_20, TD_10};  // 将TD_20和TD_10合并为另一个向量
    
    // // 合并通道，类似于 np.stack([...], axis=1)
    // cv::Mat combined(160, 160, CV_32F, cv::Scalar(0));

    // // 将 mat1 的行放入奇数行，将 mat2 的行放入偶数行
    // for (int i = 0; i < 80; ++i) {
    //     TD_10.row(i).copyTo(combined.row(i * 2));        // mat1 的奇数行
    //     TD_20.row(i).copyTo(combined.row(i * 2 + 1));    // mat2 的偶数行
    // }

    // cv::Mat output = cv::Mat::zeros(combined.size(), CV_32F); // 初始化为全0

    // // 遍历每个像素
    // for (int i = 0; i < combined.rows; ++i) {
    //     for (int j = 0; j < combined.cols; ++j) {
    //         output.at<uchar>(i, j) = (combined.at<uchar>(i, j) > 0) ? 1 : 0; // 将非零设为1，零设为0
    //     }
    // }

    // combined = combined / 255;
    // for (int i = 0; i < combined.rows; ++i) {
    //     for (int j = 0; j < combined.cols; ++j) {
    //         std::cout << static_cast<int>(combined.at<uchar>(i, j)) << " "; // 打印每个像素值
    //     }
    //     std::cout << std::endl; // 换行以便于查看
    // }

    // cv::imshow("SD", output * spatioDiffll * 255);
    // combined.convertTo(combined, CV_8U);
    // spatioDiffll = combined * spatioDiffll;
    // spatioDiffll.convertTo(spatioDiffll, CV_8U);
    // LSD_reconstructed = LSD_reconstructed.reshape(160, 160);
    // cv::imshow("LSD", LSD_reconstructed);

    // cv::resize(combined, combined, cv::Size(320, 160));

    // double minVal_c, maxVal_c;
    // cv::Point minLoc_c, maxLoc_c;
    // cv::minMaxLoc(combined, &minVal_c, &maxVal_c, &minLoc_c, &maxLoc_c);

    // std::cout << "Min Value: " << minVal_c << std::endl;
    // std::cout << "Max Value: " << maxVal_c << std::endl;

    // cv::imshow("com", combined);

    // int height = spatioDiffll.rows;//row表示行，rows表示行的总数，即图像的高
	// int width = spatioDiffll.cols;//col表示列，cols表示列的总数，即图像的宽
	//获取通道数
	// int channels = spatioDiffll.channels();
	//打印输出
	// printf("height=%d width=%d channels=%d", height, width, channels);

    // cv::imshow("LSD", LSD_reconstructed);

    // spatioDiffl = combined * spatioDiffl;
    // spatioDiffr = combined * spatioDiffr;


    tianmouc::isp::SDlSDR_2_SDxSDy(spatioDiffl,spatioDiffr,Ix,Iy);

    double minVal, maxVal;

    cv::Point minLoc, maxLoc;

    // 查找最小值和最大值
    // cv::minMaxLoc(spatioDiffl, &minVal, &maxVal, &minLoc, &maxLoc);

    // std::cout << "Min Value: " << minVal << " at (" << minLoc.x << ", " << minLoc.y << ")" << std::endl;
    // std::cout << "Max Value: " << maxVal << " at (" << maxLoc.x << ", " << maxLoc.y << ")" << std::endl;

    // cv::medianBlur(Ix, Ix, 3);
    // cv::medianBlur(Iy, Iy, 3);

    // cv::Mat conv_kernel = cv::Mat::ones(3, 3, CV_32F);
    // // conv_kernel.at<float>(3 / 2, 3 / 2) = 0;  // 设置中心元素为 0

    // cv::filter2D(Ix, Ix, -1, conv_kernel, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
    // cv::filter2D(Iy, Iy, -1, conv_kernel, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);

    // Ix = Ix / 2;
    // conv_and_threshold_inplace(Ix, 3, -2);
    // conv_and_threshold_inplace_2(Ix, 3, -30);

    // cv::absdiff(Ix, cv::Scalar(0), Ix); // 取绝对值
    // Ix = Ix <= -2;  // 使用阈值生成布尔掩码

    // Iy = Iy / 2;
    // conv_and_threshold_inplace(Iy, 3, 2);
    // conv_and_threshold_inplace_2(Iy, 3, 30);
    // cv::absdiff(Iy, cv::Scalar(0), Iy); // 取绝对值
    // Iy = Iy >= 2;  // 使用阈值生成布尔掩码

    cv::Mat result;
    cv::addWeighted(Ix, 1, Iy, 1, 0, result);
    
    // result.convertTo(result, CV_8SC1);
    // cv::medianBlur(result, result, 5); 
    
    cv::resize(result, result, cv::Size(h*2, w), cv::INTER_LINEAR);

    // double minVal, maxVal;

    // cv::Point minLoc, maxLoc;

    double lowThreshold1 = -100;
    double highThreshold1 = -1;

    double lowThreshold2 = 1;
    double highThreshold2 = 100;

    // 创建两个掩码，分别用于 -20 到 -5 和 5 到 20 的像素
    // cv::Mat mask1, mask2;
    // cv::inRange(result, lowThreshold1, highThreshold1, mask1);
    // cv::inRange(result, lowThreshold2, highThreshold2, mask2);

    // // 合并两个掩码
    // cv::Mat combinedMask;
    // cv::bitwise_or(mask1, mask2, combinedMask);

    // 使用合并的掩码保留符合条件的像素
    cv::Mat mask1, mask2;
    cv::inRange(result, lowThreshold1, highThreshold1, mask1);
    cv::inRange(result, lowThreshold2, highThreshold2, mask2);

    // 合并两个掩码
    cv::Mat combinedMask;
    cv::bitwise_or(mask1, mask2, combinedMask);

    // 使用合并的掩码保留符合条件的像素
    cv::Mat filtered;
    result.copyTo(filtered, combinedMask);

    // 归一化结果
    // cv::Mat sd;
    // cv::normalize(filtered, sd, 0, 255, cv::NORM_MINMAX, CV_32F);

    // conv_and_threshold_inplace_2(sd, 9, 1.f);

    // filtered.convertTo(filtered, CV_8UC1);

    // cv::medianBlur(sd, sd, 5);

    // double alpha = 2.0; // 对比度增益，值越大对比度越高
    // int beta = 0;       // 亮度调整

    // 应用公式 new_pixel = alpha * original_pixel + beta
    // sd.convertTo(sd, -1, alpha, beta);

    // filtered = filtered;

    return (filtered + 20) * 3;
}

cv::Mat DataController::getRGB(){

    // struct  timeval  tv1, tv2;
    // struct  timezone tz1, tz2;
    using namespace cv;
    const unsigned  height = ch/2;
    const unsigned  width = cw;
    //320 * 320 --> 640 * 320
    // int err_code = tianmouc::process::cone_reader(this->coneBuffer,
    //                                             raw,
    //                                             this->w,
    //                                             this->h);
    //std::cout<<raw.cols<<raw.rows<<std::endl;
    //string discriptionFile = "/home/lyh/rawfile.txt";
    //std::ofstream record_time_file;
    //record_time_file.open(discriptionFile, std::ios::out | std::ios::app);
    //record_time_file << raw(Range(0,10),Range(0,10))<<"\n\n";
    //std::cout<<raw(Range(0,10),Range(0,10));
    // int dataAmount = cw*ch;
    // QMetaObject::invokeMethod(this->Window,"data_decoded", Q_ARG(bool,isRod), Q_ARG(int,dataAmount),Q_ARG(int,globalCameraIdx));

    std::vector<cv::Mat> dst;
    //manual white balanced

    float r_mean = 0;//0.39556265;
    float g_mean = 0;//0.42169937;
    float b_mean = 0;//0.65865874;
    //WB coeff
    float r_std =  1/Rcor;//0.18127572356817837;
    float g_std =  1/GCor;//0.261213080783536;
    float b_std =  1/Bcor;//0.6503947447629356;
    float scaling_factor = 1.0;//255.0;
    //pixel_normed = (pixel/scaling_factor - mean) / std
    tianmouc::isp::RGBNormalizer norm(scaling_factor, r_mean,g_mean,b_mean,
                               r_std,g_std,b_std);
    long diff;

    cv::Mat demosaicRaw;
    // if(_qDEBUG_PREVIEW) gettimeofday(&tv1,&tz1);
    if(last_read_rgb_id==last_read_raw_id){
        // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
        // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
        getRaw();
        last_read_rgb_id=last_read_raw_id;
    }
    bool status = tianmouc::isp::prepocess(raw, cw, ch,dst,demosaicRaw, norm,enableAWB);
    // if(_qDEBUG_PREVIEW) gettimeofday(&tv2,&tz2);
    // if(_qDEBUG_PREVIEW) diff = 1000000 * (tv2.tv_sec - tv1.tv_sec)+ tv2.tv_usec - tv1.tv_usec;
    // if(_qDEBUG_PREVIEW) qDebug()<<"cone norm  :"<<diff<<"(us)";

    if(enableRGBHist){
        // QMetaObject::invokeMethod(this->Window,"updatePreviewHist",Q_ARG(bool,isRod));
        RGBhist = tianmouc::isp::histogramRGB(demosaicRaw,2<<9,binScaling);
    }
    cv::Mat processed;
    cv::merge(dst,processed);
    // if(this->enableFlipDisplay){
    //     cv::flip(processed,processed,1); // >0: y-, 0: x-, <0: x+y
    // }
    if(isBLCmode){
        runtimeConeBLC += processed;
        if(BLCFRAME>BLC_DENOISE_NUM){
            BLCisReady = true;
            // this->stop();
            // return;
        }
        BLCFRAME ++;
    }
    if(enableBLCDenoise){
        processed -= (runtimeConeBLC/BLCFRAME);
    }
    processed.convertTo(processed, CV_8UC3);
    cv::resize(processed, processed, cv::Size(320, 160), cv::INTER_LINEAR);
    return processed;
}

cv::Mat DataController::getRGB_block(){
    // NOTE 不建议与其他函数一起使用 因为本方法是阻塞的， 逻辑可能会有问题
    static int last_visit_cone_reader=-1;
    // struct  timeval  tv1, tv2;
    // struct  timezone tz1, tz2;
    using namespace cv;
    const unsigned  height = ch;
    const unsigned  width = 2 * cw;
    //320 * 320 --> 640 * 320
    // int err_code = tianmouc::process::cone_reader(this->coneBuffer,
    //                                             raw,
    //                                             this->w,
    //                                             this->h);
    //std::cout<<raw.cols<<raw.rows<<std::endl;
    //string discriptionFile = "/home/lyh/rawfile.txt";
    //std::ofstream record_time_file;
    //record_time_file.open(discriptionFile, std::ios::out | std::ios::app);
    //record_time_file << raw(Range(0,10),Range(0,10))<<"\n\n";
    //std::cout<<raw(Range(0,10),Range(0,10));
    // int dataAmount = cw*ch;
    // QMetaObject::invokeMethod(this->Window,"data_decoded", Q_ARG(bool,isRod), Q_ARG(int,dataAmount),Q_ARG(int,globalCameraIdx));

    std::vector<cv::Mat> dst;
    //manual white balanced

    float r_mean = 0;//0.39556265;
    float g_mean = 0;//0.42169937;
    float b_mean = 0;//0.65865874;
    //WB coeff
    float r_std =  1/Rcor;//0.18127572356817837;
    float g_std =  1/GCor;//0.261213080783536;
    float b_std =  1/Bcor;//0.6503947447629356;
    float scaling_factor = 1.0;//255.0;
    //pixel_normed = (pixel/scaling_factor - mean) / std
    tianmouc::isp::RGBNormalizer norm(scaling_factor, r_mean,g_mean,b_mean,
                               r_std,g_std,b_std);
    long diff;

    cv::Mat demosaicRaw;
    // if(_qDEBUG_PREVIEW) gettimeofday(&tv1,&tz1);
    while(last_visit_cone_reader==cone_buffer_idx){

    }
    last_visit_cone_reader=cone_buffer_idx;
    getRaw();

    // if(last_read_rgb_id==last_read_raw_id){
    //     // 每次执行 getDiff 之后执行次数变量增加，如果两者不相等 说明还没有访问过，直接输出就可以，如果相等，说明已经访问过这个时刻的了
    //     // 但是这么做仍然有问题  如果访问的频率高于tmc的帧率 就会有重复计算了
    //     getRaw();
    //     last_read_rgb_id=last_read_raw_id;
    // }
    bool status = tianmouc::isp::prepocess(raw, cw, ch,dst,demosaicRaw, norm,enableAWB);
    // if(_qDEBUG_PREVIEW) gettimeofday(&tv2,&tz2);
    // if(_qDEBUG_PREVIEW) diff = 1000000 * (tv2.tv_sec - tv1.tv_sec)+ tv2.tv_usec - tv1.tv_usec;
    // if(_qDEBUG_PREVIEW) qDebug()<<"cone norm  :"<<diff<<"(us)";

    if(enableRGBHist){
        // QMetaObject::invokeMethod(this->Window,"updatePreviewHist",Q_ARG(bool,isRod));
        RGBhist = tianmouc::isp::histogramRGB(demosaicRaw,2<<9,binScaling);
    }
    cv::Mat processed;
    cv::merge(dst, processed);
    // if(this->enableFlipDisplay){
    //     cv::flip(processed,processed,1); // >0: y-, 0: x-, <0: x+y
    // }
    if(isBLCmode){
        runtimeConeBLC += processed;
        if(BLCFRAME>BLC_DENOISE_NUM){
            BLCisReady = true;
            // this->stop();
            // return;
        }
        BLCFRAME ++;
    }
    if(enableBLCDenoise){
        processed -= (runtimeConeBLC/BLCFRAME);
    }
    processed.convertTo(processed, CV_8UC3);
    return processed;
    // cv::cvtColor(processed, processed, cv::COLOR_RGB2BGR);
    //===========================CONE AE
    // what is cone AE ?
    // if(!isBLCmode && enableConeAE){
    //     AEBuffer[globalCameraIdx]->writeConeData(processed.data,(w*2)*h*3,timestamp);
    // }
    Size dsize = Size(height, width / 2);
    if(enableFullResolution){
        dsize = Size(width, height);
    }
    cv::resize(processed, resized, dsize, 0, 0, cv::INTER_LINEAR);
    //taoyi temp add digigain
    resized *= scaling;
    // uchar *pSrc = (uchar*)resized.data;
    // if(this->isRunning){
    //     QMetaObject::invokeMethod(this->Window,"setGraphPanel",
    //                               Q_ARG(QImage,QImage(pSrc,resized.cols,resized.rows,resized.step,
    //                                                        QImage::Format_RGB888)),Q_ARG(int,0),Q_ARG(int,globalCameraIdx));
    //     if(enableRGBHist){
    //         QMetaObject::invokeMethod(this->Window,"setGraphPanel",
    //                                   Q_ARG(QImage,QImage(RGBhist.data,
    //                                                      RGBhist.cols,RGBhist.rows,RGBhist.step,
    //                                                      QImage::Format_RGB888)),Q_ARG(int,5),Q_ARG(int,globalCameraIdx));
    //     }

    // }
    return resized;
}

void DataController::loadDataFromMat(CameraFrame &dataFrame){
    if(if_fake_device!=true){
        printf("error writing data to device wrapper, as the device is a real one\n");
        return;
    }
    if(dataFrame.data_type==0){
        // 这里不能分成两个if 不然可能会出现 上一帧为cone下一帧为rod 但是下一帧的时间还没到但是被输出的情况
        rod_iolock=1;
        // 在存储的时候二维数组的[0]为左目 [1]为右目
        last_rod_time=dataFrame.timeStamp;
        memcpy(rod_pvalue[(rod_buffer_idx+1)%frame_buffer_len],dataFrame.rod_pvalue,sizeof(dataFrame.rod_pvalue));
        // timediff=dataFrame.timediff;
        // spatioDiffl=dataFrame.spatioDiffl;
        // spatioDiffr=dataFrame.spatioDiffr;
        rod_iolock=0;
        rod_buffer_idx=(rod_buffer_idx+1)%frame_buffer_len;
        // 写入完成后再加
    }
    if(dataFrame.data_type==1){
        cone_iolock=1;
        last_cone_time=dataFrame.timeStamp;
        memcpy(cone_pvalue[(cone_buffer_idx+1)%frame_buffer_len],dataFrame.cone_pvalue,sizeof(dataFrame.cone_pvalue));
        // raw=dataFrame.raw;
        cone_iolock=0;
        cone_buffer_idx=(cone_buffer_idx+1)%frame_buffer_len;
        // 写入完成后再加
    }
}
void DataController::copyRodPValue(int des[CAMERA_ROD_PVALUE_LENGTH]){
    memcpy(des,rod_pvalue[(rod_buffer_idx)%frame_buffer_len],CAMERA_ROD_PVALUE_LENGTH*sizeof(int));
}
void DataController::copyConePValue(int des[CAMERA_CONE_PVALUE_LENGTH]){
    memcpy(des,cone_pvalue[(cone_buffer_idx)%frame_buffer_len],CAMERA_CONE_PVALUE_LENGTH*sizeof(int));
}
void DataController::setFreqOutput()
{
    if_freq_output = true;
}
void DataController::unsetFreqOutput()
{
    if_freq_output = false;
}
DataController::~DataController()
{
    // delete cam_frames;
    // free(rod_pvalue);
    // free(cone_pvalue);
    lynCameraUninit(cameraHandle);
}

bool DataController::isFakeDevice(){
    return if_fake_device;
    // if true means its fake device 
}
std::vector<uint32_t> write_single_gen(uint32_t lyncam_reg_addr, uint32_t lyncam_reg_data)
{
    std::vector<uint32_t> write_iic_single;
    write_iic_single.push_back(0x40);
    write_iic_single.push_back(0xa);
    write_iic_single.push_back(0x108);
    write_iic_single.push_back(0x166);
    write_iic_single.push_back(0x108);
    write_iic_single.push_back((lyncam_reg_addr >> 8) & 0xff);
    write_iic_single.push_back(0x108);
    write_iic_single.push_back((lyncam_reg_addr) & 0xff);
    write_iic_single.push_back(0x108);
    write_iic_single.push_back((lyncam_reg_data) & 0xff);
    write_iic_single.push_back(0x108);
    write_iic_single.push_back(0x200);
    write_iic_single.push_back(0x100);
    write_iic_single.push_back(0x0d);
    return write_iic_single;
}
void test_iic(lynCameraHandle_t cameraHandle)
{
    // test write

    uint32_t lyncam_reg_addr = 0x0547;
    uint32_t lyncam_reg_data = 0x1;
    // std::vector<uint32_t>write_iic_single_vec = write_single_gen(lyncam_reg_addr, lyncam_reg_data);
    // uint32_t* iic_buf = write_iic_single_vec.data();
    // int lens = write_iic_single_vec.size();
    bool status = false;
    status = LyncamWriteByte_ONCE(cameraHandle, lyncam_reg_addr, lyncam_reg_data);
    // bool status = lynCameraWriteIIC(cameraHandle, iic_buf, lens);
}


