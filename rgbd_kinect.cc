#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>    // For fastNlMeansDenoisingColored
#include <libfreenect.h>
#include "System.h"  // ORB-SLAM3 main header


using namespace std;
using namespace cv;

class KinectDevice {
public:
    KinectDevice() : ctx(nullptr), dev(nullptr), new_rgb(false), new_depth(false), stop_thread(false) {}

    bool init() {
        if (freenect_init(&ctx, nullptr) < 0) {
            cerr << "freenect_init failed!" << endl;
            return false;
        }
        freenect_set_log_level(ctx, FREENECT_LOG_WARNING);

        if (freenect_num_devices(ctx) < 1) {
            cerr << "No Kinect found!" << endl;
            return false;
        }
        if (freenect_open_device(ctx, &dev, 0) < 0) {
            cerr << "Could not open Kinect device." << endl;
            return false;
        }
        freenect_set_user(dev, this);

        auto vid_mode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
        if (!vid_mode.is_valid) { 
            cerr << "RGB mode invalid!" << endl; 
            return false; 
        }
        freenect_set_video_mode(dev, vid_mode);

        auto dep_mode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM);
        if (!dep_mode.is_valid) { 
            cerr << "Depth mode invalid!" << endl; 
            return false; 
        }
        freenect_set_depth_mode(dev, dep_mode);

        freenect_set_video_callback(dev, videoCB);
        freenect_set_depth_callback(dev, depthCB);

        if (freenect_start_video(dev) < 0 || freenect_start_depth(dev) < 0) {
            cerr << "Failed to start Kinect streams." << endl;
            return false;
        }

        worker = thread(&KinectDevice::loop, this);
        return true;
    }

    void shutdown() {
        {
            lock_guard<mutex> lock(stop_mtx);
            stop_thread = true;
        }
        if (worker.joinable())
            worker.join();

        if (dev) {
            freenect_stop_video(dev);
            freenect_stop_depth(dev);
            freenect_close_device(dev);
        }
        if (ctx)
            freenect_shutdown(ctx);
    }

    bool grabFrames(Mat &rgb, Mat &depth) {
        lock_guard<mutex> lock(data_mtx);
        if (new_rgb && new_depth) {
            rgb = rgb_.clone();
            depth = depth_.clone();
            new_rgb = false;
            new_depth = false;
            return true;
        }
        return false;
    }

private:
    freenect_context *ctx;
    freenect_device  *dev;
    Mat rgb_, depth_;
    bool new_rgb, new_depth;
    thread worker;
    mutex data_mtx, stop_mtx;
    bool stop_thread;

    static void videoCB(freenect_device* d, void* rgb, uint32_t) {
        auto *self = static_cast<KinectDevice*>(freenect_get_user(d));
        if (!self) return;
        lock_guard<mutex> lock(self->data_mtx);
        // Create an OpenCV Mat header for the raw data (Kinect provides 640x480 RGB)
        Mat tmp(480, 640, CV_8UC3, rgb);
        // Convert from Kinect RGB (RGB order) to OpenCV BGR
        cvtColor(tmp, self->rgb_, COLOR_RGB2BGR);
        self->new_rgb = true;
    }

    static void depthCB(freenect_device* d, void* depth, uint32_t) {
        auto *self = static_cast<KinectDevice*>(freenect_get_user(d));
        if (!self) return;
        lock_guard<mutex> lock(self->data_mtx);
        // Create an OpenCV Mat header for the depth data (Kinect provides 640x480 depth in 16UC1)
        Mat tmp(480, 640, CV_16UC1, depth);
        self->depth_ = tmp.clone();
        self->new_depth = true;
    }

    void loop() {
        while (true) {
            {
                lock_guard<mutex> lock(stop_mtx);
                if (stop_thread)
                    break;
            }
            if (freenect_process_events(ctx) < 0) {
                cerr << "freenect_process_events error." << endl;
                break;
            }
            this_thread::sleep_for(chrono::milliseconds(2));
        }
    }
};

int main(int argc, char** argv) {
    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <vocab_file> <settings_file>\n";
        return 1;
    }

    // Depth factor based on calibration:
    // Kinect V1 raw depth is in millimeters; if a raw value of 2047 corresponds to 0.30 m,
    // then the conversion factor is approximately: 2047 / 0.30 ≈ 6823.33
    const float depthFactor = 6823.33f;
    cout << "[INFO] DepthMapFactor = " << depthFactor << endl;

    // Initialize ORB-SLAM3 in RGB-D mode.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

    KinectDevice kinect;
    if (!kinect.init()) {
        cerr << "Kinect init failed.\n";
        return 1;
    }
    cout << "[INFO] Kinect started successfully.\n";

    while (true) {
        Mat rgb, depth;
        if (kinect.grabFrames(rgb, depth)) {
            // --- Preprocessing Start ---
            // Preprocess the RGB image
            Mat denoisedRGB;
            fastNlMeansDenoisingColored(rgb, denoisedRGB, 10, 10, 7, 21);
            rgb = denoisedRGB;
            
            // Preprocess the Depth image
            Mat filteredDepth;
            medianBlur(depth, filteredDepth, 5);
            depth = filteredDepth;
            // --- Preprocessing End ---

            // Convert 16-bit depth (in mm) to 32-bit float (in meters)
            Mat depthFloat;
            depth.convertTo(depthFloat, CV_32FC1, 1.0f / depthFactor);

            double tframe = double(getTickCount()) / getTickFrequency();
            SLAM.TrackRGBD(rgb, depthFloat, tframe);

            // Display the RGB frame
            imshow("Kinect RGB", rgb);
            if ((waitKey(1) & 0xFF) == 27)  // Exit if ESC is pressed
                break;
        }
        this_thread::sleep_for(chrono::milliseconds(5));
    }

    kinect.shutdown();
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}

