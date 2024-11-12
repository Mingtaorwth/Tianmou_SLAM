#include "dual_camera.hpp"
#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
inline size_t readBinaryFile(const char *filename, unsigned char *&data)
{
    FILE *fp;
    fp = fopen(filename, "rb");
    if (fp == NULL)
        return 0;            // 空指针则返回0，文件打开失败
    fseek(fp, 0, SEEK_END);  // 将文件指针移动至文件末尾
    size_t size = ftell(fp); // 计算文件大小,单位:Byte
    printf("read in file size %lu\n", size);
    fseek(fp, 0, SEEK_SET); // 把文件指针移回初始位置（文件开头）
    data = (unsigned char *)malloc(sizeof(unsigned char) * size);
    fread(data, size, 1, fp);
    fclose(fp);
    return size;
}

inline size_t viewBinaryFileSize(const char *filename)
{
    FILE *fp;
    fp = fopen(filename, "rb");
    if (fp == NULL)
        return 0;            // 空指针则返回0，文件打开失败
    fseek(fp, 0, SEEK_END);  // 将文件指针移动至文件末尾
    size_t size = ftell(fp); // 计算文件大小,单位:Byte
    fseek(fp, 0, SEEK_SET);  // 把文件指针移回初始位置（文件开头）
    fclose(fp);
    return size;
}

// size：要写入的数据大小(单位：字节)
inline size_t writeBinaryFile(const char *filename, const unsigned char *data, size_t size)
{
    FILE *fp;
    fp = fopen(filename, "wb");
    if (fp == NULL)
        return 0;
    size_t written_size = fwrite(data, size, 1, fp);
    fclose(fp);
    return written_size;
}

DualCamera::DualCamera()
{
    // count device number, cannot be dismissed
    int dev = lynCameraEnumerate();
    std::cout << "Devices number: " << dev << std::endl;
}
DualCamera::~DualCamera()
{
    // 如果在这里delete 会报segment fault
    // delete cameraL;
    // delete cameraR;
}
void DualCamera::DataRecorder(double start_time, double end_time, int max_length, char *log_folder_path)
{
    auto curr_time = std::chrono::high_resolution_clock::now();
    auto init_time = std::chrono::high_resolution_clock::now();

    cameraL = new DataController(0);
    cameraR = new DataController(-1);

    cameraL->unsetFreqOutput();
    cameraR->unsetFreqOutput();

    // if(this->n_device<0){
    //     return;
    // }else if(this->n_device==2){
    //     cameraArray=new DataController[2]{DataController(0),DataController(1)};
    // }else if(this->n_device){
    //     cameraArray=new DataController[1]{DataController(0)};
    // }
    printf("-----------DataRecorder start read camera-------\n");
    // for (int i =0;i<this->n_device;i++){
    //     cameraArray[i].startRead();
    // }
    if(!cameraL->isFakeDevice()){
        cameraL->startRead(init_time);
    }
    if(!cameraR->isFakeDevice()){
        cameraR->startRead(init_time);
    }

    int curr_left_length = 0;
    int curr_right_length = 0;

    printf("-----------DataRecorder while loop-------\n");
    double last_left_cone_time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(std::chrono::high_resolution_clock::now() - init_time).count();
    double last_left_rod_time = last_left_cone_time;
    double last_righ_cone_time = last_left_cone_time;
    double last_righ_rod_time = last_left_cone_time;
    CameraFrame *frames = (CameraFrame *)malloc(sizeof(CameraFrame) * max_length * 2);
    // CameraImage images[max_length][2];
    printf("-------waiting--------\n");
    while (std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(std::chrono::high_resolution_clock::now() - init_time).count() < (start_time))
    {
        sleep(0.002);
        // printf("-------waiting--------\n");
        // curr_time = std::chrono::high_resolution_clock::now();
    }
    printf("-------reading--------\n");
    while (std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(std::chrono::high_resolution_clock::now() - init_time).count() < (end_time) and ((!cameraR->isFakeDevice() and  curr_right_length < max_length) or (!cameraL->isFakeDevice() and  curr_left_length < max_length)))
    {

        // std::cout << (std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(std::chrono::high_resolution_clock::now() - init_time).count() < (end_time - start_time)) <<" curr time period " << std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(std::chrono::high_resolution_clock::now() - init_time).count() <<" (end_time - start_time) " <<(end_time - start_time) << std::endl;

        curr_time = std::chrono::high_resolution_clock::now();

        // continue;
        // printf("!cameraL->isFakeDevice() cameraL->last_cone_time > last_left_cone_time %lf %lf ",curr_left_length < max_length, cameraL->last_cone_time, last_left_cone_time);
        if (!cameraL->isFakeDevice() and curr_left_length < max_length and cameraL->last_cone_time > last_left_cone_time)
        {
            printf("                        record cameraL cone\n");
            last_left_cone_time = cameraL->last_cone_time;
            frames[curr_left_length * 2 + 0].data_type = 1;
            // images[curr_left_length*2+0].raw=cameraL->getRaw();
            cameraL->copyConePValue(frames[curr_left_length * 2 + 0].cone_pvalue);
            frames[curr_left_length * 2 + 0].timeStamp = last_left_cone_time;
            curr_left_length += 1;
        }
        if (!cameraL->isFakeDevice() and curr_left_length < max_length and cameraL->last_rod_time > last_left_rod_time)
        {
            last_left_rod_time = cameraL->last_rod_time;
            frames[curr_left_length * 2 + 0].data_type = 0;
            // images[curr_left_length*2+0].timediff=cameraL->getTimeDiff();
            // // frames[curr_left_length*2+0].timediff=(cameraL->timediff).clone();
            // images[curr_left_length*2+0].spatioDiffl=cameraL->getSpatioDiffL();
            // // frames[curr_left_length*2+0].spatioDiffl=(cameraL->spatioDiffl).clone();
            // images[curr_left_length*2+0].spatioDiffr=cameraL->getSpatioDiffR();
            // // frames[curr_left_length*2+0].spatioDiffr=(cameraL->spatioDiffr).clone();
            cameraL->copyRodPValue(frames[curr_left_length * 2 + 0].rod_pvalue);

            frames[curr_left_length * 2 + 0].timeStamp = last_left_rod_time;
            curr_left_length += 1;
            printf("                        record cameraL rod\n");
        }

        if (!cameraR->isFakeDevice() and curr_right_length < max_length and cameraR->last_cone_time > last_righ_cone_time)
        {
            last_righ_cone_time = cameraR->last_cone_time;
            frames[curr_right_length * 2 + 1].data_type = 1;
            // images[curr_right_length*2+1].raw=cameraR->getRaw();
            // frames[curr_right_length*2+1].raw=(cameraR->raw).clone();
            frames[curr_right_length * 2 + 1].timeStamp = last_righ_cone_time;
            cameraR->copyConePValue(frames[curr_right_length * 2 + 1].cone_pvalue);

            curr_right_length += 1;
            printf("                        record cameraR cone\n");
        }
        if (!cameraR->isFakeDevice() and curr_right_length < max_length and cameraR->last_rod_time > last_righ_rod_time)
        {
            last_righ_rod_time = cameraR->last_rod_time;
            frames[curr_right_length * 2 + 1].data_type = 0;
            // images[curr_right_length*2+1].timediff=cameraR->getTimeDiff();
            // images[curr_right_length*2+1].spatioDiffl=cameraR->getSpatioDiffL();
            // images[curr_right_length*2+1].spatioDiffr=cameraR->getSpatioDiffR();
            cameraR->copyRodPValue(frames[curr_right_length * 2 + 1].rod_pvalue);

            // frames[curr_right_length][0].timediff=(cameraR->timediff).clone();
            // frames[curr_right_length][0].spatioDiffl=(cameraR->spatioDiffl).clone();
            // frames[curr_right_length][0].spatioDiffr=(cameraR->spatioDiffr).clone();
            frames[curr_right_length * 2 + 1].timeStamp = last_righ_rod_time;
            curr_right_length += 1;
            printf("                        record cameraR rod\n");
        }
    }
    printf("--------------finished data collection\n");

    // writeBinaryFile(log_path,(unsigned char*)frames,sizeof(frames));
    if (access(log_folder_path, 0))
    // 如果有文件 则返回0
    {
        mkdir(log_folder_path, S_IRWXU);
        // std::filesystem::create_directories(log_folder_path);
        std::cout << "fs create dir succ: " << log_folder_path << std::endl;
    }
    else
    {
        std::cout << "fs dir exists: " << log_folder_path << std::endl;
    }

    std::string log_frames_folder = (std::string(log_folder_path) + "/frames");
    if (access(log_frames_folder.c_str(), 0))
    {
        mkdir(log_frames_folder.c_str(), S_IRWXU);
        // std::filesystem::create_directories(log_frames_folder.c_str());
        std::cout << "fs create dir succ: " << log_frames_folder.c_str() << "|||" << std::endl;
    }
    else
    {
        std::cout << "fs dir exsits: " << log_frames_folder << std::endl;
    }
    std::ofstream fout((std::string(log_folder_path) + "/time_stamps.frm").c_str(), std::ios::out | std::ios::binary);
    if (!fout.is_open())
    {
        printf("-------------fault cannot open log file");
        return;
    }

    char img_path[128];
    printf("--------------------saving data to files");
    fout.write((char *)&max_length, sizeof(max_length));
    for (int frameIdx = 0; frameIdx < max_length*2; frameIdx++)
    {
        // for (int cameraIdx = 0; cameraIdx < 2; cameraIdx++)
        // {
            // fout.write((char *)&(frames[frameIdx]), sizeof(CameraFrame));
            fout.write((char *)&(frames[frameIdx].timeStamp), sizeof(double));
            fout.write((char *)&(frames[frameIdx].data_type), sizeof(int));
            fout.write((char *)(frames[frameIdx].rod_pvalue), sizeof(int)*CAMERA_ROD_PVALUE_LENGTH);
            fout.write((char *)(frames[frameIdx].cone_pvalue), sizeof(int)*CAMERA_CONE_PVALUE_LENGTH);
            //     fout.write((char*)&frames[frameIdx*2+cameraIdx].timeStamp,sizeof(std::chrono::high_resolution_clock::time_point));
            //     fout.write((char*)&frames[frameIdx*2+cameraIdx].data_type,sizeof(bool));

            //     // std::stringstream oss;
            //     // oss << "i=" << i
            //     //     << ", d=" << std::left << std::setw(6) << std::fixed << std::setprecision(2) << d
            //     //     << ", cs=" << cs
            //     //     << ", ss=" << ss;
            //     if(frames[frameIdx*2+cameraIdx].data_type==1){
            //         snprintf(img_path,128,"%s/raw_%02d_%04d.bmp",log_frames_folder.c_str(),cameraIdx,frameIdx);
            //         fout.write((char*)&frames[frameIdx*2+cameraIdx].cone_pvalue,sizeof(int)*CAMERA_CONE_PVALUE_LENGTH);
            //         // cv::imwrite(img_path,images[frameIdx*2+cameraIdx].raw);
            //         printf("print to %s \n",img_path);
            //     }
            //     if(frames[frameIdx*2+cameraIdx].data_type==0){
            //         fout.write((char*)&frames[frameIdx*2+cameraIdx].rod_pvalue,sizeof(int)*CAMERA_ROD_PVALUE_LENGTH);

            //         snprintf(img_path,128,"%s/timediff_%02d_%04d.bmp",log_frames_folder.c_str(),cameraIdx,frameIdx);
            //         // cv::imwrite(img_path,images[frameIdx*2+cameraIdx].timediff);
            //         printf("print to %s \n",img_path);
            //         // snprintf(img_path,128,"%s/spatioDiffl_%02d_%04d.bmp",log_frames_folder.c_str(),cameraIdx,frameIdx);
            //         // cv::imwrite(img_path,images[frameIdx*2+cameraIdx].spatioDiffl);
            //         // snprintf(img_path,128,"%s/spatioDiffr_%02d_%04d.bmp",log_frames_folder.c_str(),cameraIdx,frameIdx);
            //         // cv::imwrite(img_path,images[frameIdx*2+cameraIdx].spatioDiffr);
            //     }
        // }
        // // fout.write((char*)&(frames[idx][0]),sizeof(CameraFrame));
        // // fout.write((char*)&(frames[idx][1]),sizeof(CameraFrame));
        // // //图像数据存储为二进制流文件 src:源图像 datName:欲保存的文件名
        // // //把mat 转换 为 IplImage 原因是这种图像格式数据访问更为高效
        // // cv::IplImage
        // // //图像宽、高、通道、深度 有时我们需要放置一些头信息，因此学会这一点也是很重要的
        // // int height = img.height, width = img.width, depth = img.depth, channel = img.nChannels, dataSize = img.imageSize, widthStep = img.widthStep;
        // // //创建dat文件，其中前5*4字节为图像宽、高、通道、深度信息
        // // fout.write((char*)&width, sizeof(width));
        // // fout.write((char*)&height, sizeof(height));
        // // fout.write((char*)&depth, sizeof(depth));
        // // fout.write((char*)&channel, sizeof(channel));
        // // fout.write((char*)&dataSize, sizeof(dataSize));
        // // fout.write((char*)&widthStep, sizeof(widthStep));
        // // //写入图像像素到dat文件
        // // fout.write((char*)img.imageData, dataSize);
        // // fout.close();
    }
    printf("write %d frames, file size: \n", max_length);
    fout.close();
}
void DualCamera::DataLoader(char *log_folder_path)
{
    printf("--------------finished data collection\n");
    // writeBinaryFile(log_path,(unsigned char*)frames,sizeof(frames));
    if (access(log_folder_path, 0))
    {
        std::cout << "find log dir: " << log_folder_path << std::endl;
    }

    std::string log_frames_folder = (std::string(log_folder_path) + "frames");
    if (access(log_folder_path, 0))
    {
        std::cout << "find log dir" << log_frames_folder << std::endl;
    }
    std::ofstream fin((std::string(log_folder_path) + "./time_stamps.frm").c_str(), std::ios::in | std::ios::binary);
    if (!fin.is_open())
    {
        printf("-------------fault cannot open log file");
        return;
    }
    char img_path[128];
    printf("--------------------saving data to files");
}

void DualCamera::DataReplayLoop(char *log_folder_path)
{
    // size_t file_length=viewBinaryFileSize(log_path);
    // int max_length=file_length/2/sizeof(CameraFrame);
    // unsigned char* frames_data;
    // size_t byteSize=readBinaryFile(log_path,frames_data);
    int max_length = 0;
    // 希望这个是一个指向二维数组的指针
    // CameraFrame *frames=(CameraFrame*)frames_data;

    // std::ifstream inF;
    // inF.open(log_path, std::ifstream::binary);
    // FILE *fin = NULL;
    // fin = fopen((std::string(log_folder_path) + "/time_stamps.frm").c_str(), "rb");
    // // 数据块首地址: "&b"，元素大小: "sizeof(unsigned __int8)"， 元素个数: "10"， 文件指针："pd"
    // fread(&max_length, sizeof(int), 1, fin);
    // printf("load recorded data with length %d unit size %lu\n", max_length, sizeof(CameraFrame));
    // CameraFrame *frames = (CameraFrame *)malloc(sizeof(CameraFrame) * (max_length * 2+4));
    // // fread(frames, sizeof(CameraFrame), max_length*2, fin);
    // for (int frameIdx = 0; frameIdx < max_length*2; frameIdx++)
    // {
    //     fread(&(frames[frameIdx]), sizeof(CameraFrame), 1, fin);
    //     printf("read in %d \n",frameIdx);
    // }
    // fclose(fin);

    std::ifstream inF((std::string(log_folder_path) + "/time_stamps.frm").c_str(), std::ios::in | std::ios::binary);
    if (!inF)
    {
        std::cout << "cannot find file" << (std::string(log_folder_path) + "/time_stamps.frm").c_str() << std::endl;
    }
    std::streampos filePointerPos = inF.tellg(); //   save   current   position
    inF.seekg(0, std::ios::end);
    std::cout << "file length=" << inF.tellg() << std::endl;
    inF.seekg(filePointerPos); //   restore   saved   position

    inF.read((char *)&(max_length), sizeof(int));
    printf("load recorded data with length %d unit size %lu\n", max_length, sizeof(CameraFrame));
    // CameraFrame *frames = (CameraFrame *)malloc(sizeof(CameraFrame) * max_length * 2);

    CameraFrame *frames = (CameraFrame *)malloc(sizeof(CameraFrame) * (max_length * 2+4));
    // printf("alloc %lu size of mem\n",sizeof(frames_buffer));
    // // printf("type %d \n",frames[max_length*2+1].data_type);
    // inF.read((char *)(frames_buffer), sizeof(CameraFrame)* (max_length * 2));
    // printf("%d \n",inF.gcount());
    // CameraFrame frames[max_length];
    for (int frameIdx = 0; frameIdx < max_length*2; frameIdx++)
    {
        // for (int cameraIdx = 0; cameraIdx < 2; cameraIdx++)
        // {
            // inF.read((char *)&(frames[0]), sizeof(CameraFrame));
            // printf("%d \n",inF.gcount());
            inF.read((char *)&(frames[0].timeStamp), sizeof(double));
            printf("%d %d\n",inF.gcount(),inF.tellg());
            inF.read((char *)&(frames[0].data_type), sizeof(int));
            printf("%d %d\n",inF.gcount(),inF.tellg());
            inF.read((char *)(frames[0].rod_pvalue), sizeof(int)*CAMERA_ROD_PVALUE_LENGTH);
            printf("%d %d\n",inF.gcount(),inF.tellg());

            inF.read((char *)(frames[0].cone_pvalue), sizeof(int)*CAMERA_CONE_PVALUE_LENGTH);
            printf("%d %d\n",inF.gcount(),inF.tellg());
            // inF.read((char *)&(frames[0].cone_pvalue[CAMERA_CONE_PVALUE_LENGTH/4]), sizeof(int)*CAMERA_CONE_PVALUE_LENGTH/4);
            // printf("%d \n",inF.gcount());
            // inF.read((char *)&(frames[0].cone_pvalue[CAMERA_CONE_PVALUE_LENGTH/2]), sizeof(int)*CAMERA_CONE_PVALUE_LENGTH/4);
            // printf("%d \n",inF.gcount());
            // inF.read((char *)&(frames[0].cone_pvalue[CAMERA_CONE_PVALUE_LENGTH/4*3]), sizeof(int)*CAMERA_CONE_PVALUE_LENGTH/4);
            // printf("%d \n",inF.gcount());
            printf("read in %d \n",frameIdx);
            // sleep(0.05);
        // }
    }
    inF.close();

    printf("load recorded data with length %d unit size %lu test frame 0 data type %d \n", max_length, sizeof(CameraFrame), frames[0].data_type);


    auto init_time = std::chrono::high_resolution_clock::now();
    auto curr_time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(std::chrono::high_resolution_clock::now() - init_time).count();
    int curr_left_length = 0;
    int curr_right_length = 0;
    while (curr_left_length < max_length or curr_right_length < max_length)
    {
        curr_time = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(std::chrono::high_resolution_clock::now() - init_time).count();
        if (curr_left_length < max_length)
        {
            if (curr_time > frames[curr_left_length * 2 + 0].timeStamp)
            {
                cameraL->loadDataFromMat(frames[curr_left_length * 2 + 0]);
                std::cout << "camera id L frame " << curr_left_length << "   at time  " << curr_time<< "\033[A\r" << std::endl;
            }
            // if(curr_time>frames[curr_left_length*2+0].timeStamp and frames[curr_left_length*2+0].data_type==0){
            //     // 这里不能分成两个if 不然可能会出现 上一帧为cone下一帧为rod 但是下一帧的时间还没到但是被输出的情况
            //     cameraL->rod_iolock=1;
            //     // 在存储的时候二维数组的[0]为左目 [1]为右目
            //     cameraL->last_rod_time=frames[curr_left_length*2+0].timeStamp;
            //     cameraL->timediff=frames[curr_left_length*2+0].timediff.clone();
            //     cameraL->spatioDiffl=frames[curr_left_length*2+0].spatioDiffl.clone();
            //     cameraL->spatioDiffr=frames[curr_left_length*2+0].spatioDiffr.clone();
            //     curr_left_length+=1;
            //     cameraL->rod_iolock=0;
            // }
            // if(curr_time>frames[curr_left_length*2+0].timeStamp and frames[curr_left_length*2+0].data_type==1){
            //     cameraL->cone_iolock=1;
            //     cameraL->last_cone_time=frames[curr_left_length*2+0].timeStamp;
            //     cameraL->raw=frames[curr_left_length*2+0].raw.clone();
            //     curr_left_length+=1;
            //     cameraL->cone_iolock=0;
            // }
        }
        if (curr_right_length < max_length)
        {
            if (curr_time > frames[curr_right_length * 2 + 1].timeStamp)
            {
                cameraL->loadDataFromMat(frames[curr_right_length * 2 + 1]);
                std::cout << "camera id R frame " << curr_right_length << "   at time  " <<curr_time << "\033[A\r" << std::endl;
            }

            // if(curr_time>frames[curr_right_length*2+1].timeStamp and frames[curr_right_length*2+1].data_type==0){
            //     // 这里不能分成两个if 不然可能会出现 上一帧为cone下一帧为rod 但是下一帧的时间还没到但是被输出的情况
            //     cameraR->rod_iolock=1;
            //     // 在存储的时候二维数组的[0]为左目 [1]为右目
            //     cameraR->last_rod_time=frames[curr_right_length*2+1].timeStamp;
            //     cameraR->timediff=frames[curr_right_length*2+1].timediff.clone();
            //     cameraR->spatioDiffl=frames[curr_right_length*2+1].spatioDiffl.clone();
            //     cameraR->spatioDiffr=frames[curr_right_length*2+1].spatioDiffr.clone();
            //     curr_right_length+=1;
            //     cameraR->rod_iolock=0;
            // }
            // if(curr_time>frames[curr_right_length*2+1].timeStamp and frames[curr_right_length*2+1].data_type==1){
            //     cameraR->cone_iolock=1;
            //     cameraR->last_cone_time=frames[curr_right_length*2+1].timeStamp;
            //     cameraR->raw=frames[curr_right_length*2+1].raw.clone();
            //     curr_right_length+=1;
            //     cameraR->cone_iolock=0;
            // }
        }
    }
    free(frames);
    frames=NULL;
}
void DualCamera::DataReplay(char *log_folder_path)
{
    cameraL = new DataController(-1);
    cameraR = new DataController(-1);
    // init fake device
    std::thread replay_cam(&DualCamera::DataReplayLoop, this, log_folder_path);
    replay_cam.detach();
    // while(1){

    // }
}
void DualCamera::DataListener()
{
    auto init_time = std::chrono::high_resolution_clock::now();
    cameraL = new DataController(0);
    cameraR = new DataController(1);

    cameraL->unsetFreqOutput();
    cameraR->unsetFreqOutput();

    // if(this->n_device<0){
    //     return;
    // }else if(this->n_device==2){
    //     cameraArray=new DataController[2]{DataController(0),DataController(1)};
    // }else if(this->n_device){
    //     cameraArray=new DataController[1]{DataController(0)};
    // }
    printf("-----------DataListener start read camera-------\n");
    // for (int i =0;i<this->n_device;i++){
    //     cameraArray[i].startRead();
    // }
    cameraL->startRead(init_time);
    cameraR->startRead(init_time);    
}
void DualCamera::SingleDataListener()
{
    auto init_time = std::chrono::high_resolution_clock::now();
    cameraL = new DataController(0);
    // cameraR = new DataController(1);

    cameraL->unsetFreqOutput();
    // cameraR->unsetFreqOutput();

    // if(this->n_device<0){
    //     return;
    // }else if(this->n_device==2){
    //     cameraArray=new DataController[2]{DataController(0),DataController(1)};
    // }else if(this->n_device){
    //     cameraArray=new DataController[1]{DataController(0)};
    // }
    printf("-----------sigle DataListener start read camera-------\n");
    // for (int i =0;i<this->n_device;i++){
    //     cameraArray[i].startRead();
    // }
    cameraL->startRead(init_time);
    // cameraR->startRead(init_time);    
}