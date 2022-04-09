#ifndef VINS_MOBILE_ANDROIDPORT_VIEWCONTROLLER_H
#define VINS_MOBILE_ANDROIDPORT_VIEWCONTROLLER_H



/*
 * This is the class which corresponds to the ViewController-Objective C Class.
 * I started by copying all the code over 
 * and moved on by tweaking all the compile-errors
 * In that process i never deleted the original code but commented it out where i had replaced it
 * this was done with the intention of being able to quickly lock back into the original code
 */



#import "utility.hpp"
#import "feature_tracker.hpp"

#import "global_param.hpp"
#import "VINS.hpp"
#include <queue>
#import "draw_result.hpp"

#include "keyframe.h"
#include "loop_closure.h"
#include "keyfame_database.h"
#import <sys/utsname.h>

// added in the continous process of tranlating objective c code
#include <condition_variable> // std::condition_variable con
#include <android/log.h>

typedef double NSTimeInterval;
#define APPNAME "VINS_Android"
#include <thread>
#include <jni.h>

extern "C" {
    #include <time.h>
}

#include <android/looper.h>
#include <android/sensor.h>

struct IMU_MSG {
    NSTimeInterval header;
    Vector3d acc;
    Vector3d gyr;
};

struct IMG_MSG {
    NSTimeInterval header;
    map<int, Vector3d> point_clouds;
};

struct IMG_DATA {
    NSTimeInterval header;
    cv::Mat image; // UIImage *image;
};

struct IMG_DATA_CACHE {
    NSTimeInterval header;
    cv::Mat equ_image;
    cv::Mat image; // UIImage *image;
};

struct VINS_DATA_CACHE {
    NSTimeInterval header;
    Vector3f P;
    Matrix3f R;
};

typedef shared_ptr <IMU_MSG const > ImuConstPtr;
typedef shared_ptr <IMG_MSG const > ImgConstPtr;
//@end


class ViewController {

//@interface ViewController : UIViewController<CvVideoCameraDelegate,UITextViewDelegate>
//{
private:
    const int videoWidth = 480;
    const int videoHeight = 640;
    
    // CvVideoCamera is only for iOS
    //CvVideoCamera* videoCamera;
    bool isCapturing;
    cv::Ptr<FeatureTracker> feature_tracker;
    cv::Size frameSize;
    // never used? uint64_t prevTime;
    std::mutex _condition;          // NSCondition *_condition;
    std::thread mainLoop;          // NSThread *mainLoop;
    std::thread draw;              // NSThread *draw;
    std::thread saveData;          // NSThread *saveData;
    std::thread loop_thread;       // NSThread *loop_thread;
    std::thread globalLoopThread;  // NSThread *globalLoopThread;
//UITextView *textY;
//}

/*************************** Save data for debug ***************************/

    bool start_record = false;

    bool start_playback = false;

    bool start_playback_vins = false;

    unsigned long imageDataIndex = 0;

    unsigned long imageDataReadIndex = 0;

    unsigned long imuDataIndex = 0;

    unsigned long imuDataReadIndex = 0;

    unsigned long vinsDataIndex = 0;

    unsigned long vinsDataReadIndex = 0;

    queue<IMG_DATA> imgDataBuf;

    // purely for saving (is it?)
//    NSMutableData *imuDataBuf = [[NSMutableData alloc] init];
//
//    NSData *imuReader;
//
//    NSMutableData *vinsDataBuf = [[NSMutableData alloc] init];
//
//    NSData *vinsReader;

    IMG_DATA imgData;

    IMU_MSG imuData;

    KEYFRAME_DATA vinsData;

/*************************** Save data for debug ***************************/

/******************************* UI CONFIG *******************************/

// false:  VINS trajectory is the main view, AR image is in left bottom
// true: AR image is the main view, VINS is in left bottom
    bool ui_main = false;

    bool box_in_AR = false;

    bool box_in_trajectory = false;

// If initialized finished, start show is true
    bool start_show = false;


// Textview for showing vins status
    int loop_old_index = -1;

    float x_view_last = -5000;

    float y_view_last = -5000;

    float z_view_last = -5000;

    float total_odom = 0;

/******************************* UI CONFIG *******************************/

    FeatureTracker featuretracker;

    VINS vins;

// Store the fesature data processed by featuretracker
    queue<ImgConstPtr> img_msg_buf;

// Store the IMU data for vins
    queue<ImuConstPtr> imu_msg_buf;

// Store the IMU data for motion-only vins
    queue<IMU_MSG_LOCAL> local_imu_msg_buf;

// The number of measurements waiting to be processed
    int waiting_lists = 0;

    int frame_cnt = 0;

// Lock the feature and imu data buffer
    std::mutex m_buf;

    std::condition_variable con;

    NSTimeInterval current_time = -1;

    NSTimeInterval lateast_imu_time = -1;
public:
    NSTimeInterval getLateast_imu_time() const;

private:

    int imu_prepare = 0;

// MotionManager for read imu data
    // IMU Things:
    const int LOOPER_ID_USER = 3;
    const int SENSOR_REFRESH_RATE_HZ = 100;
    const int32_t SENSOR_REFRESH_PERIOD_US = int32_t(1000000 / SENSOR_REFRESH_RATE_HZ);

    static ASensorEventQueue *accelerometerEventQueue;
    static ASensorEventQueue *gyroscopeEventQueue;

    static int process_imu_sensor_events(int fd, int events, void* data);

    // singleton for static callbacks
    static ViewController* instance;

// Segment the trajectory using color when re-initialize
    int segmentation_index = 0;

// Set true:  30 HZ pose output and AR rendering in front-end (very low latency)
// Set false: 10 HZ pose output and AR rendering in back-end
    bool USE_PNP = true;

// Lock the solved VINS data feedback to featuretracker
    std::mutex m_depth_feedback;

// Lock the IMU data feedback to featuretracker
    std::mutex m_imu_feedback;

// Solved VINS feature feedback to featuretracker
    list<IMG_MSG_LOCAL> solved_features;

// Solved VINS status feedback to featuretracker
    VINS_RESULT solved_vins;

/******************************* Loop Closure ******************************/

// Raw image data buffer for extracting FAST feature
    queue<pair<cv::Mat, double>> image_buf_loop;

// Lock the image_buf_loop
    std::mutex m_image_buf_loop;

// Detect loop
    LoopClosure *loop_closure = nullptr;

// Keyframe database
    KeyFrameDatabase keyframe_database;

// Control the loop detection frequency
    int keyframe_freq = 0;

// Index the keyframe
    int global_frame_cnt = 0;

// Record the checked loop frame
    int loop_check_cnt = 0;

// Indicate if breif vocabulary read finish
    bool voc_init_ok = false;

// Indicate the loop frame index
    int old_index = -1;

// Translation drift
    Eigen::Vector3d loop_correct_t = Eigen::Vector3d(0, 0, 0);

// Rotation drift
    Eigen::Matrix3d loop_correct_r = Eigen::Matrix3d::Identity();

/******************************* Loop Closure ******************************/

// MARK: Unity Camera Mode Switching
// Ground truth from UI switch property "self.switchUIAREnabled"

// Implied, updated by updateCameraMode()
    bool imuPredictEnabled = false;

// Implied, updated by updateCameraMode()
    bool cameraMode = true;

// Implied, updated by updateCameraMode()
    //AR模式下这里为true
    bool imageCacheEnabled = cameraMode && !USE_PNP;
    
public:
    ViewController();
    ~ViewController();

    inline static double timeStampToSec (long long timeStamp) { return timeStamp / 1000000000.0; };
    static NSTimeInterval systemUptime();

    void testMethod(){
        //LOGI("Testmethod is working");
    }

    std::mutex viewUpdateMutex;
    std::string tvXText;
    std::string tvYText;
    std::string tvZText;
    std::string tvFeatureText;
    std::string tvTotalText{"TOTAL:"};
    std::string tvBufText;
    std::string tvLoopText{"LOOP:"};
    bool initImageVisible = true;
    
    float virtualCamDistance = 5;
    
// MARK: ViewController Methods

    void viewDidLoad();

/*
 Main process image thread: this thread detects and track feature between two continuous images
 and takes the newest VINS result and the corresponding image to draw AR and trajectory.
 */
    queue<IMG_DATA_CACHE> image_pool;
    queue<VINS_DATA_CACHE> vins_pool;
    IMG_DATA_CACHE image_data_cache;
    cv::Mat lateast_equa;
    cv::Mat lateast_image; //UIImage *lateast_image;
    Vector3f lateast_P;
    Matrix3f lateast_R;

    cv::Mat pnp_image;
    Vector3d pnp_P;
    Matrix3d pnp_R;

    /**
     * Takes RGBA doesnt change the format!
     */
    void processImage(cv::Mat& image, double timeStamp, bool isScreenRotated);


/*
 Send imu data and visual data into VINS
 */
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements();


    /**
     * Used for feature tracking, returns and removes all imu_msg from the local imu buffer
     * that are in between the last and the current header (timeStamp)
     */
    vector<IMU_MSG_LOCAL> getImuMeasurements(double header);

    void send_imu(const ImuConstPtr &imu_msg);

    //TODO: solve quick fix
    bool mainLoop_isCancelled = false;
    /**
     * VINS thread: this thread tightly fuses the visual measurements and imu data and solves pose, velocity, IMU bias, 3D feature for all frame in WINNDOW
     * If the newest frame is keyframe, then push it into keyframe database
     */
    void run();

    int kf_global_index;
    bool start_global_optimization = false;
    void process();


    //TODO: solve quick fix
    bool loop_thread_isCancelled = false;
    /**
     * Loop detection thread: this thread detect loop for newest keyframe and retrieve features
     */
    void loopDetectionLoop();

    //TODO: solve quick fix
    bool globalLoopThread_isCancelled = false;
    /**
     * GLobal Pose graph thread: optimize global pose graph based on realative pose from vins and update the keyframe database
     */
    void globalPoseGraphLoop();

    /*
     * Z^
     * |   /Y
     * |  /
     * | /
     * |/
     * --------->X
     * IMU data process and interpolation 
     */
    bool imuDataFinished = false;
    bool vinsDataFinished = false;
    shared_ptr<IMU_MSG> cur_acc = shared_ptr<IMU_MSG>(new IMU_MSG()); // shared_ptr<IMU_MSG> cur_acc(new IMU_MSG());
    vector<IMU_MSG> gyro_buf;  // for Interpolation  仅有两个数据
    void imuStartUpdate();
    void imuStopUpdate();
    void onStop();

/********************************************************************UI View Controler********************************************************************/
    void showInputView();

    // Where is this coming from (delegate)? It's not being called anywhere.
    void showOutputImage(cv::Mat* image) {
//-(void)showOutputImage:(UIImage*)image
//{
        //TODO: [featureImageView setImage:image];
    }
/********************************************************************UI View Controler********************************************************************/


/********************************************************************UI Button Controler********************************************************************/


    // TODO: UI Interaction
    /// switch UI between graph(VINS) and camera(AR)
    void switchUI(bool isChecked);
    /// loop button
    void loopButtonPressed(bool isChecked);


    //TODO: solve quick fix
    bool saveData_isCancelled = false;
    void saveDataLoop();

    void tapSaveImageToIphone(cv::Mat* image) {

    }

    void recordImu() {

    }

    void recordVins() {

    }

    void recordImageTime(IMG_DATA& image_data) {

    }

    void recordImage(IMG_DATA& image_data) {

    }

    /**
     * tries to read image time from file and writes it into property imgData.header
     */
    bool readImageTime(unsigned long index) {
        return false;

    }

    /**
     * tries to read image from file and writes it into property imgData.image
     */
    bool readImage(unsigned long index) {

    }

/**************************************************************About record and playback data for debug**********************************************************/

    // TODO: call when app closes
    void viewDidDisappear() {

        if (isCapturing)
        {
            // TODO: stop the camera [videoCamera stop];
        }
        mainLoop_isCancelled = true; // [mainLoop cancel];
        // TODO: where does it get started? [draw cancel];
#ifdef LOOP_CLOSURE
        loop_thread_isCancelled = true; // [loop_thread cancel];
#endif
    }

    void viewDidUnload() {

    }

    void dealloc() {

    }

/*
 Check the device
 */
    DeviceType deviceName();

    // rewrite this check for required android api level 
    bool iosVersion();
//@end

};


#endif //VINS_MOBILE_ANDROIDPORT_VIEWCONTROLLER_H
