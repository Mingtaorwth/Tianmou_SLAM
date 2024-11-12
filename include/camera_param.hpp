#ifndef _WGOMOTION_CAMERA_PARAM
#define _WGOMOTION_CAMERA_PARAM
    #define CAMERA_CONE_H 320
    #define CAMERA_CONE_W 320
    #define CAMERA_ROD_H 160
    #define CAMERA_ROD_W 160
    #define CAMERA_ROD_BUFFER_LENGTH (106496)
    #define CAMERA_ROD_PVALUE_LENGTH (44448)
    #define CAMERA_CONE_PVALUE_LENGTH (0x1A000)
    #define CAMERA_FRAMES_BUFFER_LENGTH (10)

    // readfile  time gap
    // is related to the CPU clock freq
    #define ROD_GAP (1900)//(950) //us
    #define RGB_GAP (32000)//(32000)//us

    //denoising
    #define BLC_DENOISE_NUM 15

    // TIME related constant
    #define GUI_REFRESH_ms 33
    #define SAFE_BUTTON_PUSH_GAP 5 //ms
    #define WAITTIME_AVOID_OVERLOOP 5 //us

    // PCIE related const
    #define ROD_WORDLENGTH_CONST 4
    #define PCIE_SEND_ROD_BATCH_SIZE 5
    #define PCIE_SEND_CONE_BATCH_SIZE 1

    //NN=======
    #define MAX_QUEUE_LEN_ROD 250
    #define MAX_QUEUE_LEN_CONE 10

    #define INFER_ROD_BATCH_SIZE 3
    #define INFER_CONE_BATCH_SIZE 1

    #define TIMESTAMP_INVALID 0xffffffffffffffff
    #define SINGLE_FILE_SIZE 106496
    #define IMGSIZE (160*320*4)
    #define USE_BYTE_TRACK true
    //use opencv
    #ifndef USE_OPEN_CV
    #define USE_OPEN_CV
    #endif
#endif