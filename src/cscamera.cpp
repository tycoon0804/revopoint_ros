/*******************************************************************************
* This file is part of the 3DViewer                                            *
*                                                                              *
* Copyright (C) 2022 Revopoint3D Company Ltd.                                  *
* All rights reserved.                                                         *
*                                                                              *
* This program is free software: you can redistribute it and/or modify         *
* it under the terms of the GNU General Public License as published by         *
* the Free Software Foundation, either version 3 of the License, or            *
* (at your option) any later version.                                          *
*                                                                              *
* This program is distributed in the hope that it will be useful,              *
* but WITHOUT ANY WARRANTY; without even the implied warranty of               *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)             *
* for more details.                                                            *
*                                                                              *
********************************************************************************/

#include <typeinfo>
#include <chrono>
#include <cassert>
#include <Windows.h>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "../include/cscamera.h"
#include "../include/cameraparaid.h"

//#include <QDebug>
//#include <QSize>
//#include <QVariant>
//#include <QThread>
//#include <QRectF>
//#include <QTimer>
//#include <QMetaEnum>

#include <algorithm>
#include <3DCamera.hpp>

static CSRange DEPTH_RANGE_LIMIT = { 0, 65535 };
static CSRange CAMERA_HDR_LEVEL_RANGE = { 2,8 };
#define HDR_SCALE_DEFAULT  5

#ifdef _DEBUG
    #define _CS_DEBUG_
#endif

using namespace cs;
using namespace cs::parameter;

//publishers for image and pointcloud
image_transport::Publisher pup_color, pub_depth;
ros::Publisher pcpub;
ros::Publisher rgb_info;
ros::Publisher depth_info;

sensor_msgs::CameraInfo rgb_camera_info_;
sensor_msgs::CameraInfo depth_camera_info_;

#define MEATER_OF_MM 1000

//QMetaEnum CSCamera::metaEnum = QMetaEnum::fromType<CAMERA_PARA_ID>();

static const std::map<int, const char*> STREAM_FORMAT_MAP =
{ 
    { STREAM_FORMAT_MJPG,    "MJPG" },
    { STREAM_FORMAT_RGB8,    "RGB8" },
    { STREAM_FORMAT_Z16,     "Z16" },
    { STREAM_FORMAT_Z16Y8Y8, "Z16Y8Y8" },
    { STREAM_FORMAT_PAIR,    "PAIR" },
    { STREAM_FORMAT_H264,    "H264" },
    { STREAM_FORMAT_I8DS,    "I8DS" },
    { STREAM_FORMAT_XZ32,    "XZ32" },
    { STREAM_FORMAT_GRAY,    "GRAY" }
};

const std::map<int, const char*> CSCamera::AUTO_EXPOSURE_MODE_MAP =
{
    { AUTO_EXPOSURE_MODE_CLOSE,             "Close"},
    { AUTO_EXPOSURE_MODE_FIX_FRAMETIME,     "Speed First"},
    { AUTO_EXPOSURE_MODE_HIGH_QUALITY,      "Quality First"},
    { AUTO_EXPOSURE_MODE_FORE_GROUND,       "Foreground"}
};

const std::map<int, const char*> CSCamera::FILTER_TYPE_MAP =
{
    { FILTER_CLOSE,      "Close"},
    { FILTER_SMOOTH,     "Smooth"},
    { FILTER_MEDIAN,     "Median"},
    { FILTER_TDSMOOTH,   "TDSmooth"}
};

static const std::map<int, CSRange> FILTER_RANGE_MAP =
{
    { FILTER_CLOSE,      { 0, 0 }},
    { FILTER_SMOOTH,     { 3, 9 }},
    { FILTER_MEDIAN,     { 3, 5 }},
    { FILTER_TDSMOOTH,   { 3, 7 }}
};

struct ParaInfo 
{
    int type;   //  0 : STREAM_TYPE_DEPTH, 1 : STREAM_TYPE_RGB
    int id;     //  PROPERTY_TYPE
};

static const std::map<CAMERA_PARA_ID, ParaInfo> CAMERA_PROPERTY_MAP =
{
    { PARA_DEPTH_GAIN,              { STREAM_TYPE_DEPTH , PROPERTY_GAIN} },
    { PARA_DEPTH_EXPOSURE,          { STREAM_TYPE_DEPTH , PROPERTY_EXPOSURE} },
    { PARA_DEPTH_FRAMETIME,         { STREAM_TYPE_DEPTH , PROPERTY_FRAMETIME} },
    

    { PARA_RGB_GAIN,                { STREAM_TYPE_RGB ,   PROPERTY_GAIN} },
    { PARA_RGB_AUTO_EXPOSURE,       { STREAM_TYPE_RGB ,   PROPERTY_ENABLE_AUTO_EXPOSURE} },
    { PARA_RGB_AUTO_WHITE_BALANCE,  { STREAM_TYPE_RGB ,   PROPERTY_ENABLE_AUTO_WHITEBALANCE} },
    { PARA_RGB_WHITE_BALANCE,       { STREAM_TYPE_RGB ,   PROPERTY_WHITEBALANCE} }
};

static const std::map<CAMERA_PARA_ID, int> CAMERA_EXTENSION_PROPERTY_MAP =
{
    { PARA_DEPTH_AUTO_EXPOSURE,    PROPERTY_EXT_AUTO_EXPOSURE_MODE },
    { PARA_DEPTH_THRESHOLD,        PROPERTY_EXT_CONTRAST_MIN },
    { PARA_DEPTH_HDR_MODE,         PROPERTY_EXT_HDR_MODE },
    { PARA_DEPTH_HDR_SETTINGS,     PROPERTY_EXT_HDR_EXPOSURE },
    { PARA_DEPTH_HDR_LEVEL,        PROPERTY_EXT_HDR_SCALE_SETTING },
    { PARA_DEPTH_SCALE,            PROPERTY_EXT_DEPTH_SCALE },  
    { PARA_DEPTH_ROI,              PROPERTY_EXT_DEPTH_ROI },
    { PARA_DEPTH_RANGE,            PROPERTY_EXT_DEPTH_RANGE },
    { PARA_TRIGGER_MODE,           PROPERTY_EXT_TRIGGER_MODE },
    { PARA_CAMERA_IP,              PROPERTY_EXT_CAMERA_IP },
    { PARA_RGB_EXPOSURE,           PROPERTY_EXT_EXPOSURE_TIME_RGB }
};

static const std::vector<CAMERA_PARA_ID> CAMERA_OPTION_PARA_LIST =
{
    PARA_DEPTH_STREAM_FORMAT,
    PARA_DEPTH_RESOLUTION,
    PARA_DEPTH_AUTO_EXPOSURE,
    PARA_DEPTH_FILTER_TYPE,
    PARA_RGB_STREAM_FORMAT,
    PARA_RGB_RESOLUTION
};

const std::map<CAMERA_HDR_MODE, const char*> CSCamera::CAMERA_HDR_MAP =
{
    { HDR_MODE_CLOSE,       "Close" },
    { HDR_MODE_SHINE,       "Shiny" },
    { HDR_MODE_DARK,        "Dark" },
    { HDR_MODE_BOTH,        "Both" },
    { HDR_MODE_MANUAL,      "Manual" }
};

static const std::map<CAMERA_PARA_ID, nonstd::any> CAMERA_DEFAULT_STREAM_TYPE =
{
    { PARA_DEPTH_STREAM_FORMAT,   (int)STREAM_FORMAT_Z16 },
    { PARA_DEPTH_RESOLUTION,      cv::Size(640, 400) },
    { PARA_RGB_STREAM_FORMAT,     (int)STREAM_FORMAT_RGB8 },
    { PARA_RGB_RESOLUTION,        cv::Size(1280, 800) }
};

static const std::map<CAMERA_PARA_ID, nonstd::any> CAMERA_DEFAULT_PARA_VALUE =
{
    { PARA_DEPTH_RANGE,          std::pair<float, float>( 50.0f, 2000.0f ) },
    { PARA_DEPTH_GAIN,           1.0f },
    { PARA_DEPTH_EXPOSURE,       7000.0f },
    { PARA_TRIGGER_MODE,         (int)TRIGGER_MODE_OFF}
};

int CSCamera::setCameraChangeCallback(CameraChangeCallback callback, void* userData)
{
    return cs::getSystemPtr()->setCameraChangeCallback(callback, userData);
}

int CSCamera::setCameraAlarmCallback(CameraAlarmCallback callback, void* userData)
{
    return cs::getSystemPtr()->setCameraAlarmCallback(callback, userData);
}

void CSCamera::setSdkLogPath(std::string path)
{
    setLogSavePath(path.c_str());
    enableLoging(true);
}

void CSCamera::queryCameras(std::vector<CameraInfo>& cameras)
{
    cs::getSystemPtr()->queryCameras(cameras);
}

template <typename T>
static void computePointcloud(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& color_msg, void* usrData)
{
    static float c_depthScale;
    static Intrinsics c_depthIntr;
    static Intrinsics c_rgbIntr;
    static Extrinsics c_extrColor = [&](void* usrData) -> Extrinsics {
        CSCamera* camera = (CSCamera*)usrData;
        nonstd::any tmp;
        
        camera->getCameraPara(PARA_DEPTH_SCALE, tmp);
        c_depthScale = nonstd::any_cast<float>(tmp);

        camera->getCameraPara(PARA_DEPTH_INTRINSICS, tmp);
        c_depthIntr = nonstd::any_cast<Intrinsics>(tmp);

        camera->getCameraPara(PARA_RGB_INTRINSICS, tmp);
        c_rgbIntr = nonstd::any_cast<Intrinsics>(tmp);

        camera->getCameraPara(PARA_EXTRINSICS, tmp);
        return nonstd::any_cast<Extrinsics>(tmp);
    } (usrData);
    
    cs::Pointcloud pc;

    T* depth_p = (T*)const_cast<uchar*>(&depth_msg->data[0]);

    pc.generatePoints<T>(depth_p, depth_msg->width, depth_msg->height, c_depthScale, &c_depthIntr, &c_rgbIntr, &c_extrColor);

    const auto vertices = pc.getVertices();
    const auto texcoords = pc.getTexcoords();
    //const auto normals = pc.getNormals();

    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2());
    cloud_msg->header = depth_msg->header;  // Use depth image time stamp
    //cloud_msg->header.stamp = ros::Time::now(); // time
    //cloud_msg->header.frame_id = "camera_link";
    cloud_msg->height = depth_msg->height;
    cloud_msg->width = depth_msg->width;
    cloud_msg->is_dense = true;
    // cloud_msg->is_dense = false;
    // cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    //const int meater_of_mm = 1000;
    uint8_t* texture = (uint8_t*)const_cast<uchar*>(&color_msg->data[0]);
    int textureHeight =  color_msg->height;
    int textureWidth = color_msg->width;

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
    
    for (int i = 0, pc_size = pc.size(); i < pc_size; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
    {
        *iter_x = vertices[i].x / MEATER_OF_MM;
        *iter_y = vertices[i].y / MEATER_OF_MM;
        *iter_z = vertices[i].z / MEATER_OF_MM;

        if (texture)
        {
            int x, y;
            x = int(texcoords[i].u * textureWidth);
            if (x >= textureWidth) x = textureWidth - 1;
            if (x < 0)	x = 0;
            y = int(texcoords[i].v * textureHeight);
            if (y >= textureHeight) y = textureHeight - 1;
            if (y < 0)	y = 0;
            uint8_t *color = texture + (y * textureWidth + x) * 3;
            *iter_r = color[0];
            *iter_g = color[1];
            *iter_b = color[2];
        }
    }
    
    pcpub.publish(cloud_msg);
}

static void depthCallback(cs::IFramePtr frame, void* usrData)
{
    CSCamera* camera = (CSCamera*)usrData;
    
    static FrameData frameData;
    camera->onProcessFrame(TYPE_DEPTH, frame, *frameData.data);

    //emit camera->framedDataUpdated(frameData);
    if (frameData.data->size() > 0){
        
        StreamData* tmp_StreamData = &(frameData.data->front());

        cv::Mat TempMat = cv::Mat(tmp_StreamData->dataInfo.height, tmp_StreamData->dataInfo.width, CV_16UC1, tmp_StreamData->m_data.data());
        std_msgs::Header header; // empty header
        header.stamp = ros::Time::now(); // time
        header.frame_id = "camera_link"; // frame id as neededby you
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "16UC1", TempMat).toImageMsg();
        //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", TempMat).toImageMsg();
        pub_depth.publish(msg);

        depth_camera_info_.header.stamp = header.stamp;
        depth_info.publish(depth_camera_info_);

        std::vector<StreamData>().swap(*(frameData.data));
    }
}

static void rgbCallback(cs::IFramePtr frame, void* usrData)
{
    CSCamera* camera = (CSCamera*)usrData;

    static FrameData frameData;
    camera->onProcessFrame(TYPE_RGB, frame, *frameData.data);

    //emit camera->framedDataUpdated(frameData);
    if (frameData.data->size() > 0){
        //int wb = ((frameData.data->data()->dataInfo.width * 24 + 31) / 32) * 4;
        
        StreamData* tmp_StreamData = &(frameData.data->front());

        static STREAM_FORMAT f_StreamFormat = tmp_StreamData->dataInfo.format;
        if(f_StreamFormat == STREAM_FORMAT_RGB8){
            cv::Mat TempMat = cv::Mat(tmp_StreamData->dataInfo.height, tmp_StreamData->dataInfo.width, CV_8UC3, tmp_StreamData->m_data.data());
            std_msgs::Header header; // empty header
            header.stamp = ros::Time::now(); // time
            header.frame_id = "camera_link"; // frame id as neededby you
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "rgb8", TempMat).toImageMsg();
            //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", TempMat).toImageMsg();
            pup_color.publish(msg);

            rgb_camera_info_.header.stamp = header.stamp;
            rgb_info.publish(rgb_camera_info_);
        }
        else if(f_StreamFormat == STREAM_FORMAT_MJPG){
            cv::Mat TempMat = cv::imdecode(tmp_StreamData->m_data, cv::IMREAD_UNCHANGED);
            std_msgs::Header header; // empty header
            header.stamp = ros::Time::now(); // time
            header.frame_id = "camera_link"; // frame id as neededby you
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "rgb8", TempMat).toImageMsg();
            //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", TempMat).toImageMsg();
            pup_color.publish(msg);

            rgb_camera_info_.header.stamp = header.stamp;
            rgb_info.publish(rgb_camera_info_);
        }

        std::vector<StreamData>().swap(*(frameData.data));
    }
}

CSCamera::StreamThread::StreamThread(CSCamera& camera)
    : m_camera(camera)
{
    //setObjectName("StreamThread");
    lk = std::unique_lock<std::mutex>(this->m_Mutex);
    m_StreamThread = nullptr;
}

void CSCamera::StreamThread::requestInterruption()
{
    lk.mutex()->lock();
    m_isInterruptionRequested.store(true);
}

void CSCamera::StreamThread::run() const
{
    while (!m_isInterruptionRequested.load())
    {
        FrameData frameData;
        auto ret = m_camera.onGetFrame(frameData);

        if (frameData.data->size() > 0)
        {
            frameData.rgbIntrinsics = m_camera.m_rgbIntrinsics;
            frameData.depthIntrinsics = m_camera.m_depthIntrinsics;
            frameData.extrinsics = m_camera.m_extrinsics;
            frameData.depthScale = m_camera.m_depthScale;


            StreamData* tmp_StreamData = &(frameData.data->front());

            cv::Mat TempMat = cv::Mat(tmp_StreamData->dataInfo.height, tmp_StreamData->dataInfo.width, CV_16UC1, tmp_StreamData->m_data.data());
            std_msgs::Header header; // empty header
            header.stamp = ros::Time::now(); // time
            header.frame_id = "camera_link";

            //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", TempMat).toImageMsg();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "16UC1", TempMat).toImageMsg();
            pub_depth.publish(msg);

            //emit m_camera.framedDataUpdated(frameData);

            std::vector<StreamData>().swap(*(frameData.data));
        }

        if (ret == ERROR_DEVICE_NOT_CONNECT)
        {
            // If the return value indicates that the camera is not connected, add a delay.
            std::this_thread::sleep_for(constexpr const std::chrono::milliseconds(200));
        }
    }
    lk.mutex()->unlock();
}

void CSCamera::StreamThread::start()
{
    m_isInterruptionRequested.store(false);
    
    ROS_WARN("m_StreamThread thread run");
    m_StreamThread = new std::thread(&StreamThread::run, this);

    //m_StreamThread = std::thread(&StreamThread::run, this);
    //m_StreamThread = std::thread( [this] { this->run(); } );
}

void CSCamera::StreamThread::waitJoin()
{
    if(m_StreamThread && m_isInterruptionRequested.load()){
        if(cv_.wait_for(lk, constexpr const std::chrono::milliseconds(2000)) == std::cv_status::timeout)
            TerminateThread(m_StreamThread->native_handle(), 1);
        m_StreamThread->join();
    }
}

bool CSCamera::StreamThread::isRunning()
{
    if(m_StreamThread){
        return m_StreamThread->joinable();
    }
    return false;
}

CSCamera::StreamThread::~StreamThread()
{
    requestInterruption();
    waitJoin();
    delete m_StreamThread;
    ROS_DEBUG("~StreamThread");
}

CSCamera::CSCamera()
    : m_cameraState(CAMERA_DISCONNECTED)
    , m_cameraPtr(cs::getCameraPtr())
    , m_filterValue(0)
    , m_filterType(0)
    , m_fillHole(false)
    , m_hasIrStream(false)
    , m_hasDepthStream(false)
    , m_isRgbStreamSup(false)
    , m_isDepthStreamSup(false)
    , m_hdrMode(HDR_MODE_CLOSE)
    , m_hdrTimes(2)
    , m_streamThread(new StreamThread(*this))
    , m_cameraThread(nullptr)
    , m_cachedDepthExposure(0)
    , m_cachedDepthGain(0)
    , m_triggerMode(TRIGGER_MODE_OFF)
    , m_depthScale(0.1f)
{
    m_manualHdrSetting.count = 0;
}

CSCamera::~CSCamera()
{
    doDisconnectCamera();
    delete m_streamThread;
    m_streamThread = nullptr;
    ROS_DEBUG("~CSCamera");
}

bool CSCamera::restartStream()
{
    ROS_WARN("begin restart stream.");
    
    // start stream after stop stream
    bool result = stopStream();

    ROS_WARN("sleep for");
    std::this_thread::sleep_for(constexpr const std::chrono::milliseconds(200));
    result &= startStream();

    ROS_WARN("restartStream end!");
    return result;
}

bool CSCamera::startStream()
{
    //bool suc = true;
    //suc &= (bool)connect(this, &CSCamera::updateParaSignal, this, &CSCamera::onParaUpdated);
    //ROS_ASSERT(suc);

    setCameraState(CAMERA_STARTING_STREAM);
    bool result = true;
    if (m_isDepthStreamSup)
    {
        ROS_WARN("begin start depth stream");
        result &= startDepthStream();
        if (!result)
        {
            ROS_ASSERT(false);
            ROS_WARN("Error : start depth stream failed.");
        }
        ROS_WARN("start depth stream end");
    }

    if (m_isRgbStreamSup)
    {
        ROS_WARN("begin start rgb stream");
        result &= startRgbStream();
        if (!result)
        {
            ROS_ASSERT(false);
            ROS_WARN("Error : start rgb stream failed.");
        }
        ROS_WARN("start rgb stream end");
    }

    if (!result)
    {
        ROS_WARN("Error : start stream failed.");
        setCameraState(CAMERA_START_STREAM_FAILED);
        return false;
    }

    ROS_WARN("startStreamThread()");
    // start get frame thread
    startStreamThread();

    ROS_WARN("onStreamStarted()");
    onStreamStarted();

    // notify camera stream started
    setCameraState(CAMERA_STARTED_STREAM);

    return result;
}

void CSCamera::onStreamStarted()
{
    // set default para
    ROS_WARN("set camera default para");
    //for (auto key : CAMERA_DEFAULT_PARA_VALUE.keys())
    for (auto const& element : CAMERA_DEFAULT_PARA_VALUE)
    {
        setCameraPara(element.first, element.second);
    }

    // get depth scale
    PropertyExtension propExt;
    ERROR_CODE ret = m_cameraPtr->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, propExt);
    m_depthScale = propExt.depthScale;

    DEPTH_RANGE_LIMIT.max = 65535 * m_depthScale;
}

bool CSCamera::stopStream()
{
    bool result = true;

    //disconnect(this, &CSCamera::updateParaSignal, this, &CSCamera::onParaUpdated);

    ROS_WARN("stop get frame thread");
    // stop get frame thread
    stopStreamThread();

    if (m_isDepthStreamSup)
    {
        result &= stopDepthStream();
        if (!result)
        {
            ROS_ASSERT(false);
            ROS_WARN("Error : stop depth stream failed.");
        }
    }

    if (m_isRgbStreamSup)
    {
        result &= stopRgbStream();
        if (!result)
        {
            ROS_ASSERT(false);
            ROS_WARN("Error : stop rgb stream failed.");
        }
    }

    m_hasIrStream = false;
    m_hasDepthStream = false;

    setCameraState(CAMERA_STOPPED_STREAM);
    return true;
}

bool CSCamera::startDepthStream()
{
    ERROR_CODE ret;

    // get stream info by depthFormat and depthResolution
    StreamInfo info = getDepthStreamInfo();

    // you can also use the callback method
    ret = m_cameraPtr->startStream(STREAM_TYPE_DEPTH, info, depthCallback, this);
    //ret = m_cameraPtr->startStream(STREAM_TYPE_DEPTH, info, nullptr, this);
    if (ret != SUCCESS)
    {
        ROS_WARN("camera start depth stream failed(%d)!", ret);
        ROS_ASSERT(false);
        return false;
    }
    else
    {
        ROS_WARN("start depth format:%2d, width:%4d, height:%4d, fps:%2.1f", info.format, info.width, info.height, info.fps);
    }

    m_hasIrStream = m_isDepthStreamSup && ((info.format == STREAM_FORMAT_Z16Y8Y8) || (info.format == STREAM_FORMAT_PAIR));
    m_hasDepthStream = m_isDepthStreamSup && (info.format != STREAM_FORMAT_PAIR);
    ROS_INFO("start depth stream sucess");

    return ret == SUCCESS;
}

bool CSCamera::startRgbStream()
{
    ERROR_CODE ret;

    // get stream info by rgbFormat and rgbResolution
    StreamInfo info = getRgbStreamInfo();

    // you can also use the callback method
    ret = m_cameraPtr->startStream(STREAM_TYPE_RGB, info, rgbCallback, this);
    //ret = m_cameraPtr->startStream(STREAM_TYPE_RGB, info, nullptr, this);
    if (ret != SUCCESS)
    {
        ROS_DEBUG("camera start rgb stream failed(%d)!", ret);
        ROS_ASSERT(false);
        return false;
    }
    ROS_INFO("started rgb stream");

    return ret == SUCCESS;
}

bool CSCamera::stopDepthStream()
{
    ERROR_CODE ret = m_cameraPtr->stopStream(STREAM_TYPE_DEPTH);
    if (ret != SUCCESS)
    {
        ROS_WARN("camera stop depth stream failed(%d)!", ret);
        return false;
    }

    return true;
}

bool CSCamera::stopRgbStream()
{
    ERROR_CODE ret = m_cameraPtr->stopStream(STREAM_TYPE_RGB);
    if (ret != SUCCESS)
    {
        ROS_WARN("camera stop rgb stream failed(%d)!", ret);
        return false;
    }

    return true;
}

bool CSCamera::restartCamera()
{
    ROS_INFO_STREAM("CSCamera: restart camera");
    setCameraState(CAMERA_RESTARTING_CAMERA);

    //disconnect(this, &CSCamera::updateParaSignal, this, &CSCamera::onParaUpdated);
    stopStreamThread();

    ERROR_CODE ret = m_cameraPtr->restart();
    if (ret != SUCCESS)
    {
        ROS_INFO_STREAM("camera restart failed, ret = " << ret);
        ROS_ASSERT(false);
        setCameraState(CAMERA_CONNECTFAILED);
        return false;
    }

    ROS_INFO("CSCamera: restart end, wait connect...");
    
    return true;
}

bool CSCamera::connectCamera(CameraInfo info)
{
    ROS_INFO_STREAM("connectCamera, camera seiral : "<< info.serial);

    m_cameraInfo.cameraInfo = info;

    setCameraState(CAMERA_CONNECTING);

    ERROR_CODE ret = m_cameraPtr->connect(info);
    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("camera connect failed, error code : " << ret);
        setCameraState(CAMERA_CONNECTFAILED);
        return false;
    }

    // judge if depth stream is supported or not
    ret = m_cameraPtr->isStreamSupport(STREAM_TYPE_DEPTH, m_isDepthStreamSup);
    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("camera call is support stream failed, error code : " << ret);
        ROS_ASSERT(false);
    }

    // judge if RGB stream is supported or not
    ret = m_cameraPtr->isStreamSupport(STREAM_TYPE_RGB, m_isRgbStreamSup);
    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("camera call is support stream failed, error code : " << ret);
        ROS_ASSERT(false);
    }

    initDefaultStreamInfo();

    initCameraInfo();

    setCameraState(CAMERA_CONNECTED);

    return  ret == SUCCESS;
}

bool CSCamera::connectCamera()
{
    setCameraState(CAMERA_CONNECTING);

    ERROR_CODE ret = m_cameraPtr->connect();
    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("camera connect failed, error code : " << ret);
        setCameraState(CAMERA_CONNECTFAILED);
        return false;
    }

    CameraInfo info;
    ret = m_cameraPtr->getInfo(info);
    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("camera get info failed, error code : " << ret);
        setCameraState(CAMERA_CONNECTFAILED);
        return false;
    }
    m_cameraInfo.cameraInfo = info;
    ROS_INFO_STREAM("connectCamera, camera seiral : "<< m_cameraInfo.cameraInfo.serial);

    // judge if depth stream is supported or not
    ret = m_cameraPtr->isStreamSupport(STREAM_TYPE_DEPTH, m_isDepthStreamSup);

    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("camera call is support stream failed, error code : " << ret);
        ROS_ASSERT(false);
    }

    // judge if RGB stream is supported or not
    ret = m_cameraPtr->isStreamSupport(STREAM_TYPE_RGB, m_isRgbStreamSup);

    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("camera call is support stream failed, error code : " << ret);
        ROS_ASSERT(false);
    }

    initDefaultStreamInfo();

    initCameraInfo();

    setCameraState(CAMERA_CONNECTED);

    return  ret == SUCCESS;
}

bool CSCamera::reconnectCamera()
{
    ROS_INFO("CSCamera, begin reconnect camera");
    bool result = connectCamera(m_cameraInfo.cameraInfo);
    ROS_INFO("CSCamera, reconnect camera end");
    
    return result;
}

bool CSCamera::disconnectCamera()
{
    ROS_INFO("CSCamera, begin disconenct camera");
    //disconnect(this, &CSCamera::updateParaSignal, this, &CSCamera::onParaUpdated);

    doDisconnectCamera();
    ROS_INFO("CSCamera, disconenct camera end");

    return true;
}

void CSCamera::doDisconnectCamera()
{
    if (getCameraState() == CAMERA_STARTED_STREAM || getCameraState() == CAMERA_PAUSED_STREAM)
    {
        ROS_INFO_STREAM("CSCamera, begin  stop stream");
        stopStream();
    }

    setCameraState(CAMERA_DISCONNECTING);
    
    // disconnect camera
    ERROR_CODE ret = m_cameraPtr->disconnect();
    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("disconnect camera failed, error code : " << ret);
        ROS_ASSERT(false);
        setCameraState(CAMERA_DISCONNECTFAILED);
    }
    else
    {
        setCameraState(CAMERA_DISCONNECTED);
    }

    m_isRgbStreamSup = false;
    m_isDepthStreamSup = false;
}

bool CSCamera::pauseStream()
{
    if (getCameraState() != CAMERA_STARTED_STREAM) {
        ROS_WARN_STREAM("please pause stream latter, state = " << getCameraState());
        return false;
    }

    stopStreamThread();

    ERROR_CODE ret = SUCCESS;
    //depth 
    if (m_isDepthStreamSup)
    {
        ret = m_cameraPtr->pauseStream(STREAM_TYPE_DEPTH);
        if (ret != SUCCESS)
        {
            ROS_ASSERT(false);
            ROS_WARN_STREAM("pause depth stream failed, error code : " << ret);
        }
    }
     
    //rgb
    if (m_isRgbStreamSup)
    {
        ret = m_cameraPtr->pauseStream(STREAM_TYPE_RGB);
        if (ret != SUCCESS)
        {
            ROS_ASSERT(false);
            ROS_WARN_STREAM("pause rgb stream failed, error code : " << ret);
        }
    }

    setCameraState(CAMERA_PAUSED_STREAM);

    return ret == SUCCESS;
}

bool CSCamera::resumeStream()
{
    if (getCameraState() != CAMERA_PAUSED_STREAM) {
        ROS_INFO_STREAM("please resume stream latter, state = " << getCameraState());
        return false;
    }

    //depth 
    ERROR_CODE ret = SUCCESS;
    if (m_isDepthStreamSup)
    {
        ret = m_cameraPtr->resumeStream(STREAM_TYPE_DEPTH);
        if (ret != SUCCESS)
        {
            ROS_ASSERT(false);
            ROS_WARN_STREAM("resume depth stream failed, error code : " << ret);
        }
    }

    //rgb
    if (m_isRgbStreamSup)
    {
        ret = m_cameraPtr->resumeStream(STREAM_TYPE_RGB);
        if (ret != SUCCESS)
        {
            ROS_ASSERT(false);
            ROS_WARN_STREAM("resume rgb stream failed, error code : " << ret);
        }
    }

    startStreamThread();
    setCameraState(CAMERA_STARTED_STREAM);

    return ret == SUCCESS;
}

bool CSCamera::softTrigger()
{
    if (getCameraState() != CAMERA_STARTED_STREAM)
    {
        ROS_WARN("");
        return false;
    }

    int frameCount = (m_filterType == FILTER_TDSMOOTH) ? m_filterValue : 1;
  
    bool result = true;

    FrameData frameData;

    for(int i = 0; i < frameCount; i++)
    {
        ERROR_CODE ret = m_cameraPtr->softTrigger();
        if (ret != SUCCESS)
        {
            ROS_WARN("camera soft trigger failed(%d)!", ret);
            result = false;
        }

        doGetFrame(*frameData.data, 3000);
    }

    // notify
    if (frameData.data->size() > 0)
    {
        frameData.rgbIntrinsics = m_rgbIntrinsics;
        frameData.depthIntrinsics = m_depthIntrinsics;
        frameData.extrinsics = m_extrinsics;
        frameData.depthScale = m_depthScale;

        //emit framedDataUpdated(frameData);
        std::vector<StreamData>().swap(*(frameData.data));
    }

    return result;
}

void CSCamera::initCameraInfo()
{
    CameraType cameraType = getCameraTypeBySN(m_cameraInfo.cameraInfo.serial);
    m_cameraInfo.model = getCameraTypeName(cameraType);

    CS_SDK_VERSION* sdkVersion = nullptr;
    getSdkVersion(&sdkVersion);

    if (sdkVersion && sdkVersion->version)
    {
        m_cameraInfo.sdkVersion = sdkVersion->version;
    }

    //connect type
    m_cameraInfo.connectType = isNetworkConnect(m_cameraInfo.cameraInfo.uniqueId) ? CONNECT_TYPE_NET : CONNECT_TYPE_USB;

    m_hasIrStream = m_isDepthStreamSup && ((m_depthFormat == STREAM_FORMAT_Z16Y8Y8) || (m_depthFormat == STREAM_FORMAT_PAIR));
    m_hasDepthStream = m_isDepthStreamSup && (m_depthFormat != STREAM_FORMAT_PAIR);

    ERROR_CODE ret = SUCCESS;
    if (m_isDepthStreamSup)
    {
        ret = m_cameraPtr->getIntrinsics(STREAM_TYPE_DEPTH, m_depthIntrinsics);
        if (ret != SUCCESS)
        {
            ROS_WARN_STREAM("camera get depth intrinsics failed, error code : " << ret);
            ROS_ASSERT(false);
        }

        ret = m_cameraPtr->getDistort(STREAM_TYPE_DEPTH, m_depthDistort);
        if (ret != SUCCESS)
        {
            ROS_WARN_STREAM("camera get depth distortion model failed, error code : " << ret);
            ROS_ASSERT(false);
        }
    }

    if (m_isRgbStreamSup)
    {
        ret = m_cameraPtr->getIntrinsics(STREAM_TYPE_RGB, m_rgbIntrinsics);
        if (ret != SUCCESS)
        {
            ROS_WARN_STREAM("camera get rgb intrinsics failed, error code : " << ret);
            ROS_ASSERT(false);
        }

        ret = m_cameraPtr->getDistort(STREAM_TYPE_RGB, m_rgbDistort);
        if (ret != SUCCESS)
        {
            ROS_WARN_STREAM("camera get rgb distortion model failed, error code : " << ret);
            ROS_ASSERT(false);
        }

        PropertyExtension proExt;
        proExt.depthRgbMatchParam.iRgbOffset = 0;
        proExt.depthRgbMatchParam.iDifThreshold = 0;
        ret = m_cameraPtr->setPropertyExtension(PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM, proExt);
        if (ret != SUCCESS)
        {
            ROS_WARN_STREAM("camera set depth rgb match para failed, error code : " << ret);
            ROS_ASSERT(false);
        }
    }

    ret = m_cameraPtr->getExtrinsics(m_extrinsics);
    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("camera get extrinsics failed, error code : " << ret);
        ROS_ASSERT(false);
    }
}

void CSCamera::initDefaultStreamInfo()
{
    ERROR_CODE ret;
    std::vector<StreamInfo> streamInfos;

    // default depth stream
    if (m_isDepthStreamSup)
    {
        ret = m_cameraPtr->getStreamInfos(STREAM_TYPE_DEPTH, streamInfos);
        if (ret != SUCCESS || streamInfos.empty())
        {
            ROS_DEBUG("camera get stream info failed(%d)!\n", ret);
        }
        else
        {
            // format
            auto defaultFromat = (STREAM_FORMAT)nonstd::any_cast<int>(CAMERA_DEFAULT_STREAM_TYPE.at(PARA_DEPTH_STREAM_FORMAT));
            auto it = std::find_if(streamInfos.begin(), streamInfos.end()
                , [=](const StreamInfo& info)
                {
                    return defaultFromat == info.format;
                });

            it = (it == streamInfos.end()) ? streamInfos.begin() : it;
            m_depthFormat = it->format;

            // resolution
            auto defaultRes = nonstd::any_cast<cv::Size>(CAMERA_DEFAULT_STREAM_TYPE.at(PARA_DEPTH_RESOLUTION));
            auto itRes = std::find_if(streamInfos.begin(), streamInfos.end()
                , [=](const StreamInfo& info)
                {
                    return (defaultRes.width == info.width && defaultRes.height == info.height)
                        && (m_depthFormat == info.format);
                });

            // use *it if not find the resolution
            itRes = (itRes == streamInfos.end()) ? it : itRes;
            m_depthResolution = cv::Size(itRes->width, itRes->height);
        }
    }

    if (m_isRgbStreamSup)
    {
        // default rgb stream
        streamInfos.clear();
        ret = m_cameraPtr->getStreamInfos(STREAM_TYPE_RGB, streamInfos);
        if (ret != SUCCESS || streamInfos.empty())
        {
            ROS_WARN("camera get stream info failed(%d)!\n", ret);
        }
        else
        {
            // format
            auto defaultFromat = (STREAM_FORMAT)nonstd::any_cast<int>(CAMERA_DEFAULT_STREAM_TYPE.at(PARA_RGB_STREAM_FORMAT));
            auto it = std::find_if(streamInfos.begin(), streamInfos.end()
                , [=](const StreamInfo& info)
                {
                    return defaultFromat == info.format;
                });

            it = (it == streamInfos.end()) ? streamInfos.begin() : it;
            m_rgbFormat = it->format;

            // resolution
            auto defaultRes = nonstd::any_cast<cv::Size>(CAMERA_DEFAULT_STREAM_TYPE.at(PARA_RGB_RESOLUTION));
            auto itRes = std::find_if(streamInfos.begin(), streamInfos.end()
                , [=](const StreamInfo& info)
                {
                    return (defaultRes.width == info.width && defaultRes.height == info.height)
                        && (m_rgbFormat == info.format);
                });

            // use *it if not find the resolution
            itRes = (itRes == streamInfos.end()) ? it : itRes;
            m_rgbResolution = cv::Size(itRes->width, itRes->height);
        }
    }
}

int CSCamera::doGetFrame(std::vector<StreamData>& streamDatas, int timeout)
{
    auto ret = SUCCESS;

    if (m_isRgbStreamSup)
    {
        IFramePtr depthFrame, rgbFrame;

        ret = m_cameraPtr->getPairedFrame(depthFrame, rgbFrame, timeout);
        if (ret != SUCCESS)
        {
            return ret;
        }

        onProcessFrame(TYPE_DEPTH, depthFrame, streamDatas);
        onProcessFrame(TYPE_RGB, rgbFrame, streamDatas);
    }
    else
    {
        IFramePtr depthFrame;
        ret = m_cameraPtr->getFrame(STREAM_TYPE_DEPTH, depthFrame, timeout);
        if (ret != SUCCESS)
        {
            return ret;
        }

        onProcessFrame(TYPE_DEPTH, depthFrame, streamDatas);
    }

    return SUCCESS;
}

// use getPairedFrame if has a RGB camera, or use getFrame
int CSCamera::onGetFrame(FrameData& frameData, int timeout)
{
    return doGetFrame(*frameData.data, timeout);
}

bool CSCamera::onProcessFrame(STREAM_DATA_TYPE streamDataType, const IFramePtr& frame, std::vector<StreamData>& streamDatas)
{
    if (!frame || frame->empty())
    {
        return false;
    }
    
    StreamDataInfo streamDataInfo = { streamDataType, frame->getFormat(),frame->getWidth(), frame->getHeight(), frame->getTimeStamp()};

    int dataSize = frame->getSize();

    //copy data
    std::vector<unsigned char> data(dataSize);
    
    memcpy(&(data.front()), frame->getData(), dataSize);
    
    StreamData streamData = { streamDataInfo, data };

    streamDatas.push_back(streamData);

    return true;
}

void CSCamera::setCameraThread(std::thread* thread)
{
    if(thread){
        m_cameraThread = thread;
    }
    else{
        ROS_ASSERT(false);
    }
}

StreamInfo CSCamera::getDepthStreamInfo()
{
    StreamInfo info = { STREAM_FORMAT_COUNT, 0, 0, 0.0f};

    std::vector<StreamInfo> streamInfos;
    ERROR_CODE ret = m_cameraPtr->getStreamInfos(STREAM_TYPE_DEPTH, streamInfos);
    if (ret != SUCCESS || streamInfos.empty())
    {
        ROS_WARN("camera get depth stream info failed(%d)!\n", ret);
        return info;
    }

    info.fps = -1;
    for (const auto& sInfo : streamInfos)
    {
        if (sInfo.format == m_depthFormat && sInfo.width == m_depthResolution.width
            && sInfo.height == m_depthResolution.height && sInfo.fps > info.fps)
        {
            info = sInfo;
        }
    }

    return info;
}

StreamInfo CSCamera::getRgbStreamInfo()
{
    StreamInfo info = { STREAM_FORMAT_COUNT, 0, 0, 0.0f };

    std::vector<StreamInfo> streamInfos;
    ERROR_CODE ret = m_cameraPtr->getStreamInfos(STREAM_TYPE_RGB, streamInfos);
    if (ret != SUCCESS || streamInfos.empty())
    {
        ROS_WARN("camera get rgb stream info failed(%d)!\n", ret);
        return info;
    }

    info.fps = -1;
    for (const auto& sInfo : streamInfos)
    {
        if (sInfo.format == m_rgbFormat && sInfo.width == m_rgbResolution.width
            && sInfo.height == m_rgbResolution.height && sInfo.fps > info.fps)
        {
            info = sInfo;
        }
    }

    return info;
}

void CSCamera::getUserParaPrivate(CAMERA_PARA_ID paraId, nonstd::any& value)
{
    switch (paraId)
    {
    case PARA_DEPTH_STREAM_FORMAT:
        value = (int)m_depthFormat;
        break;
    case PARA_DEPTH_RESOLUTION:
        value = m_depthResolution;
        break;
    case PARA_DEPTH_FILTER:
        value = m_filterValue;
        break;
    case PARA_DEPTH_FILTER_TYPE:
        value = m_filterType;
        break;
    case PARA_DEPTH_FILL_HOLE:
        value = m_fillHole;
        break;
    case PARA_RGB_STREAM_FORMAT:
        value = (int)m_rgbFormat;
        break;
    case PARA_RGB_RESOLUTION:
        value = m_rgbResolution;
        break;
    case PARA_HAS_RGB:
        value = m_isRgbStreamSup;
        break;
    case PARA_DEPTH_HAS_IR:
        value = m_hasIrStream;
        break;
    case PARA_HAS_DEPTH:
        value = m_hasDepthStream;
        break;
    case PARA_DEPTH_INTRINSICS:
        value = m_depthIntrinsics;
        break;
    case PARA_RGB_INTRINSICS:
        value = m_rgbIntrinsics;
        break;
    case PARA_EXTRINSICS:
        value = m_extrinsics;
        break;
    case PARA_DEPTH_DISTORT:
        value = m_depthDistort;
        break;
    case PARA_RGB_DISTORT:
        value = m_rgbDistort;
        break;
    default:
        ROS_DEBUG_STREAM("unknow camera para : " << paraId);
        break;
    }
}

void CSCamera::setUserParaPrivate(CAMERA_PARA_ID paraId, nonstd::any value)
{
    switch (paraId)
    {
    case PARA_DEPTH_STREAM_FORMAT:
        setDepthFormat((STREAM_FORMAT)nonstd::any_cast<int>(value));
        return;
    case PARA_DEPTH_RESOLUTION:
        setDepthResolution(nonstd::any_cast<cv::Size>(value));
        return;
    case PARA_DEPTH_FILTER:
        setDepthFilterValue(nonstd::any_cast<int>(value));
        return;
    case PARA_DEPTH_FILTER_TYPE:
        if (m_filterType == nonstd::any_cast<int>(value))
        {
            return;
        }
        m_filterType = nonstd::any_cast<int>(value);
        break;
    case PARA_DEPTH_FILL_HOLE:
        if (m_fillHole == nonstd::any_cast<bool>(value))
        {
            return;
        }
        m_fillHole = nonstd::any_cast<bool>(value);
        break;
    case PARA_RGB_STREAM_FORMAT:
        setRgbFormat((STREAM_FORMAT)nonstd::any_cast<int>(value));
        return;
    case PARA_RGB_RESOLUTION:
        setRgbResolution(nonstd::any_cast<cv::Size>(value));
        return; 
    default:
        ROS_DEBUG_STREAM("unknow camera para : " << paraId);
        break;
    }

    //emit cameraParaUpdated(paraId, value);
}

void CSCamera::getUserParaRangePrivate(CAMERA_PARA_ID paraId, nonstd::any& min, nonstd::any& max, nonstd::any& step)
{
    switch (paraId)
    {
    case PARA_DEPTH_RANGE:
        min = DEPTH_RANGE_LIMIT.min;
        max = DEPTH_RANGE_LIMIT.max;
        step = 1;
        break;
    case PARA_DEPTH_FILTER:
        min = FILTER_RANGE_MAP.at(m_filterType).min;
        max = FILTER_RANGE_MAP.at(m_filterType).max;
        step = 1;
        break;
    default:
        ROS_DEBUG_STREAM("range does not exist, para : " << paraId);
        break;
    }
}

void CSCamera::getCameraPara(CAMERA_PARA_ID paraId, nonstd::any& value)
{
    //if (CAMERA_PROPERTY_MAP.contains(paraId))
    if (CAMERA_PROPERTY_MAP.count(paraId) > 0)
    {
        getPropertyPrivate(paraId, value);
        return;
    }

    if (CAMERA_EXTENSION_PROPERTY_MAP.count(paraId) > 0)
    {
        getExtensionPropertyPrivate(paraId, value);
        return;
    }

    getUserParaPrivate(paraId, value);
}

void CSCamera::setCameraPara(CAMERA_PARA_ID paraId, nonstd::any value)
{
    if (CAMERA_PROPERTY_MAP.count(paraId) > 0)
    {
        setPropertyPrivate(paraId, value);

        onParaUpdatedDelay(paraId, 200);
    }
    else if (CAMERA_EXTENSION_PROPERTY_MAP.count(paraId) > 0)
    {
        setExtensionPropertyPrivate(paraId, value);

        onParaUpdatedDelay(paraId, 200);
    }
    else
    {
        setUserParaPrivate(paraId, value);
    }
    
    // parameter linkage response
    onParaLinkResponse(paraId, value);
}

void CSCamera::onParaLinkResponse(CAMERA_PARA_ID paraId, const nonstd::any& value)
{
    switch (paraId)
    {
    case PARA_DEPTH_FILTER_TYPE:
        //emit cameraParaRangeUpdated(PARA_DEPTH_FILTER);
        setDepthFilterValue(FILTER_RANGE_MAP.at(m_filterType).min);
        break;
    case PARA_DEPTH_HDR_MODE: 
        // close HDR
        if (nonstd::any_cast<int>(value) == HDR_MODE_CLOSE)
        {
            restoreExposureGain();
        }
        else 
        {
            setCameraPara(PARA_DEPTH_HDR_LEVEL, m_hdrTimes);
        }
        break;
    case PARA_DEPTH_HDR_LEVEL:
        if (m_hdrMode == HDR_MODE_MANUAL)
        {
            nonstd::any value;
            getCameraPara(PARA_DEPTH_HDR_SETTINGS, value);
            setCameraPara(PARA_DEPTH_HDR_SETTINGS, value);
        }
        else
        {
            onParaUpdatedDelay(PARA_DEPTH_HDR_SETTINGS, 7000);
        }
        break;
    case PARA_TRIGGER_MODE:
        onTriggerModeChanged(nonstd::any_cast<int>(value) == TRIGGER_MODE_SOFTWAER);
        break;
    case PARA_CAMERA_IP:
    {
        // update cameraInfo when ip changed
        nonstd::any ip;
        getCameraPara(PARA_CAMERA_IP, ip);

        CameraIpSetting cameraIp = nonstd::any_cast<CameraIpSetting>(ip);
        //QString ipStr = QString("%1.%2.%3.%4").arg(cameraIp.ipBytes[0]).arg(cameraIp.ipBytes[1]).arg(cameraIp.ipBytes[2]).arg(cameraIp.ipBytes[3]);
        std::string ipStr
            = std::to_string(cameraIp.ipBytes[0]) + '.' + std::to_string(cameraIp.ipBytes[1]) + '.' + std::to_string(cameraIp.ipBytes[2]) + '.' + std::to_string(cameraIp.ipBytes[3]);
        strncpy(m_cameraInfo.cameraInfo.uniqueId, ipStr.c_str(), sizeof(m_cameraInfo.cameraInfo.uniqueId));
        break;
    }
    default:
        break;
    }
}

void CSCamera::onParaUpdated(int paraId)
{
    CAMERA_PARA_ID cameraParaId = (CAMERA_PARA_ID)paraId;
    nonstd::any value;
    getCameraPara(cameraParaId, value);
    //emit cameraParaUpdated(cameraParaId, value);
}

// get para after delayMS millisecond
void CSCamera::onParaUpdatedDelay(CAMERA_PARA_ID paraId, int delayMS)
{
    std::thread([paraId, delayMS, this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(delayMS));
        //updateParaSignal(paraId);
    }).detach();
}

void CSCamera::restoreExposureGain()
{
    std::thread([this](int afterMS = 3000) {
        std::this_thread::sleep_for(std::chrono::milliseconds(afterMS));

        ROS_INFO_STREAM("close HDR, then restore exposure : " << m_cachedDepthExposure << ", gain : " << m_cachedDepthGain);
        setPropertyPrivate(PARA_DEPTH_EXPOSURE, m_cachedDepthExposure);
        setPropertyPrivate(PARA_DEPTH_GAIN, m_cachedDepthGain);
    }).detach();
}

void CSCamera::getCameraParaRange(CAMERA_PARA_ID paraId, nonstd::any& min, nonstd::any& max, nonstd::any& step)
{
    if (CAMERA_PROPERTY_MAP.count(paraId) > 0)
    {
        //skip some property
        if (paraId == PARA_RGB_AUTO_EXPOSURE || paraId == PARA_RGB_AUTO_WHITE_BALANCE)
        {
            return;
        }

        getPropertyRangePrivate(paraId, min, max, step);
        return;
    }

    if (CAMERA_EXTENSION_PROPERTY_MAP.count(paraId) > 0)
    {
        getExtensionPropertyRangePrivate(paraId, min, max, step);
        return;
    }

    getUserParaRangePrivate(paraId, min, max, step);
}

void CSCamera::getCameraParaItems(CAMERA_PARA_ID paraId, std::list<std::pair<std::string, nonstd::any>>& list)
{
    switch (paraId)
    {
    case PARA_DEPTH_STREAM_FORMAT:
        getFromats(STREAM_TYPE_DEPTH, list);
        break;
    case PARA_DEPTH_RESOLUTION:
        getResolutions(STREAM_TYPE_DEPTH, list);
        break;
    case PARA_DEPTH_AUTO_EXPOSURE:
        getAutoExposureModes(list);
        break;
    case PARA_DEPTH_FILTER_TYPE:
        getFilterTypes(list);
        break;
    case PARA_RGB_STREAM_FORMAT:
        getFromats(STREAM_TYPE_RGB, list);
        break;
    case PARA_RGB_RESOLUTION:
        getResolutions(STREAM_TYPE_RGB, list);
        break;
    case PARA_DEPTH_HDR_MODE:
        getHdrModes(list);
        break;
    case PARA_DEPTH_HDR_LEVEL:
        getHdrLevels(list);
        break;
    case PARA_DEPTH_GAIN:
    case PARA_RGB_GAIN:
        getGains(paraId, list);
        break;
    default:
        break;
    }
}

void CSCamera::getPropertyPrivate(CAMERA_PARA_ID paraId, nonstd::any& value)
{
    if (getCameraState() != CAMERA_STARTED_STREAM)
    {
        ROS_WARN("get property failed, stream not started.");
        return;
    }

    float v = 0;

    STREAM_TYPE  streamType = (STREAM_TYPE)CAMERA_PROPERTY_MAP.at(paraId).type;
    PROPERTY_TYPE propertyType = (PROPERTY_TYPE)CAMERA_PROPERTY_MAP.at(paraId).id;

    ERROR_CODE ret = m_cameraPtr->getProperty(streamType, propertyType, v);
    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("get camera property failed, paraId:" << paraId << ", ret:" << ret);
        ROS_ASSERT(false);
    }
    value = v;

#ifdef  _CS_DEBUG_
    ROS_DEBUG_STREAM("getPropertyPrivate, paraId : " << paraId << ", v  = " << v);
#endif
}

void CSCamera::setPropertyPrivate(CAMERA_PARA_ID paraId, nonstd::any value)
{
#ifdef  _CS_DEBUG_
    ROS_DEBUG_STREAM("setPropertyPrivate, paraId : " << paraId << ", value  = " << nonstd::any_cast<float>(value));
#endif

    STREAM_TYPE  streamType = (STREAM_TYPE)CAMERA_PROPERTY_MAP.at(paraId).type;
    PROPERTY_TYPE propertyType = (PROPERTY_TYPE)CAMERA_PROPERTY_MAP.at(paraId).id;

    float valueF = nonstd::any_cast<float>(value);

    // set frame time before set depth exposure
    if (paraId == PARA_DEPTH_EXPOSURE)
    {
        updateFrametime(valueF);
    }

    ERROR_CODE ret = m_cameraPtr->setProperty(streamType, propertyType, valueF);

    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("set the camera property failed, paraId:" << paraId << ",ret:" << ret << ",value:" << valueF);
        ROS_ASSERT(false);
    }
}

void CSCamera::getPropertyRangePrivate(CAMERA_PARA_ID paraId, nonstd::any& min, nonstd::any& max, nonstd::any& step)
{
    float minf = 0, maxf = 0, stepf = 1;

    STREAM_TYPE  streamType = (STREAM_TYPE)CAMERA_PROPERTY_MAP.at(paraId).type;
    PROPERTY_TYPE propertyType = (PROPERTY_TYPE)CAMERA_PROPERTY_MAP.at(paraId).id;

    ERROR_CODE ret = m_cameraPtr->getPropertyRange(streamType, propertyType, minf, maxf, stepf);
    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("get camera property range failed, paraId:" << paraId << ",ret:" << ret);
        ROS_ASSERT(false);
        return;
    }

    min = minf;
    max = maxf;
    step = stepf;
}

void CSCamera::getExtensionPropertyPrivate(CAMERA_PARA_ID paraId, nonstd::any& value)
{
    PROPERTY_TYPE_EXTENSION type = (PROPERTY_TYPE_EXTENSION)CAMERA_EXTENSION_PROPERTY_MAP.at(paraId);

    PropertyExtension propExt;
    memset(&propExt, 0, sizeof(propExt));

    ERROR_CODE ret = m_cameraPtr->getPropertyExtension(type, propExt);
    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("get the camera extension property failed, paraId:" << paraId << ",ret:" << ret);
        ROS_ASSERT(false);
        return;
    }

    switch (type)
    {
    case PROPERTY_EXT_AUTO_EXPOSURE_MODE:
        value = (int)propExt.autoExposureMode;
        break;
    case PROPERTY_EXT_CONTRAST_MIN:
        value = (int)propExt.algorithmContrast;
        break;
    case PROPERTY_EXT_HDR_MODE: 
        {
            getHdrMode((int)propExt.hdrMode);
            value = m_hdrMode;
        }
        break;
    case PROPERTY_EXT_HDR_EXPOSURE:
        if (m_hdrMode != HDR_MODE_CLOSE)
        {
            if (m_hdrMode == HDR_MODE_MANUAL)
            {
                //add new settings
                for (int i = propExt.hdrExposureSetting.count; i < m_hdrTimes; i++)
                {
                    HdrExposureParam param;
                    param.gain = 1;
                    param.exposure = 7000;

                    propExt.hdrExposureSetting.param[i] = param;
                }

                propExt.hdrExposureSetting.count = m_hdrTimes;
            }

            value = propExt.hdrExposureSetting;  
        }   
        break;
    case PROPERTY_EXT_HDR_SCALE_SETTING: 
        {
            HdrScaleSetting scaleSetting = propExt.hdrScaleSetting;
            getHdrTimes(scaleSetting);
            value = m_hdrTimes;
            break;
        }
    case PROPERTY_EXT_DEPTH_SCALE:
        value = propExt.depthScale;
        break;  
    case PROPERTY_EXT_DEPTH_ROI: 
    {
        cv::Rect roiRect;
        roiRect.tl() = cv::Point(propExt.depthRoi.left * 1.0 / 100, propExt.depthRoi.top * 1.0 / 100);
        roiRect.br() = cv::Point(propExt.depthRoi.right * 1.0 / 100, propExt.depthRoi.bottom * 1.0 / 100);
        value = roiRect;
        break;
    }
    case PROPERTY_EXT_DEPTH_RANGE:
        value = std::pair<float, float>{ (float)propExt.depthRange.min, (float)propExt.depthRange.max};
        break;
    case PROPERTY_EXT_TRIGGER_MODE:
        value = (int)propExt.triggerMode;
        break;
    case PROPERTY_EXT_CAMERA_IP:
    {
        value = propExt.cameraIp;
        break;
    }
    case PROPERTY_EXT_EXPOSURE_TIME_RGB:
        value = propExt.uiExposureTime;
        break;
    default:
        ROS_ASSERT(false);
        break;
    }
}

void CSCamera::setExtensionPropertyPrivate(CAMERA_PARA_ID paraId, nonstd::any value)
{
    PROPERTY_TYPE_EXTENSION type = (PROPERTY_TYPE_EXTENSION)CAMERA_EXTENSION_PROPERTY_MAP.at(paraId);

    PropertyExtension propExt;
    memset(&propExt, 0, sizeof(propExt));

    switch (type)
    {
    case PROPERTY_EXT_AUTO_EXPOSURE_MODE:
        propExt.autoExposureMode = (AUTO_EXPOSURE_MODE)nonstd::any_cast<int>(value);
        break;
    case PROPERTY_EXT_CONTRAST_MIN:
        propExt.algorithmContrast = nonstd::any_cast<int>(value);
        break;
    case PROPERTY_EXT_HDR_MODE:
    {
        // cached exposure and gain
        if (m_hdrMode == HDR_MODE_CLOSE && nonstd::any_cast<int>(value) != HDR_MODE_CLOSE)
        {
            nonstd::any tmp;
            getCameraPara(PARA_DEPTH_EXPOSURE, tmp);
            m_cachedDepthExposure = std::round(nonstd::any_cast<float>(tmp));

            getCameraPara(PARA_DEPTH_GAIN, tmp);
            m_cachedDepthGain = std::round(nonstd::any_cast<float>(tmp));

            ROS_DEBUG_STREAM("HDR is close, cached depth exposure : " << m_cachedDepthExposure << ", gain : " << m_cachedDepthGain);
        }
        // update HDR mode
        m_hdrMode = (CAMERA_HDR_MODE)nonstd::any_cast<int>(value);
        int autoHdrMode = getAutoHdrMode(m_hdrMode);
        propExt.hdrMode = (HDR_MODE)autoHdrMode;
        break;
    }
    case PROPERTY_EXT_HDR_EXPOSURE:
        propExt.hdrExposureSetting = nonstd::any_cast<HdrExposureSetting>(value);
        break;
    case PROPERTY_EXT_HDR_SCALE_SETTING:
    {
        HdrScaleSetting settings;
        setHdrTimes(settings, nonstd::any_cast<int>(value));
        propExt.hdrScaleSetting = settings;
        break;
    }
    case PROPERTY_EXT_DEPTH_ROI:
    {
        cv::Rect roiRect = nonstd::any_cast<cv::Rect>(value);
        propExt.depthRoi.top = roiRect.tl().y * 100;
        propExt.depthRoi.left = roiRect.tl().x * 100;
        propExt.depthRoi.right = roiRect.br().x * 100;
        propExt.depthRoi.bottom = roiRect.br().y * 100;
        break;
    }
    case PROPERTY_EXT_DEPTH_RANGE:
    {
        auto range = nonstd::any_cast<std::pair<float, float>>(value);
        propExt.depthRange.min = std::round(range.first);
        propExt.depthRange.max = std::round(range.second);
        break;
    }
    case PROPERTY_EXT_TRIGGER_MODE:
        propExt.triggerMode = (TRIGGER_MODE)nonstd::any_cast<int>(value);
        break;
    case PROPERTY_EXT_CAMERA_IP:
        propExt.cameraIp = nonstd::any_cast<CameraIpSetting>(value);
        break;
    case PROPERTY_EXT_EXPOSURE_TIME_RGB:
        propExt.uiExposureTime = nonstd::any_cast<float>(value);
        break;
    default:
        ROS_ASSERT(false);
        return;
    }

    ERROR_CODE ret = m_cameraPtr->setPropertyExtension(type, propExt);
    if (ret != SUCCESS)
    {
        ROS_WARN_STREAM("set the camera extension property failed," << "paraId:" << paraId << ",ret:" << ret);
        ROS_ASSERT(false);
    }
}

void CSCamera::getExtensionPropertyRangePrivate(CAMERA_PARA_ID paraId, nonstd::any& min, nonstd::any& max, nonstd::any& step)
{
    switch (paraId)
    {
    case PARA_DEPTH_THRESHOLD:
        min = 0;
        max = 40;
        step = 1;
        break;
    case PARA_DEPTH_RANGE:
        min = DEPTH_RANGE_LIMIT.min;
        max = DEPTH_RANGE_LIMIT.max;
        step = 1;
        break;
    case PARA_DEPTH_HDR_LEVEL:
        min = CAMERA_HDR_LEVEL_RANGE.min;
        max = CAMERA_HDR_LEVEL_RANGE.max;
        step = 1;
        break;
    case PARA_RGB_EXPOSURE:
    {
        PropertyExtension propExt;
        ERROR_CODE ret = m_cameraPtr->getPropertyExtension(PROPERTY_EXT_EXPOSURE_TIME_RANGE_RGB, propExt);
        if (ret != SUCCESS)
        {
            ROS_WARN_STREAM("get the camera extension property failed, type:" << PROPERTY_EXT_EXPOSURE_TIME_RANGE_RGB << ",ret:" << ret);
            ROS_ASSERT(false);
        }
        else 
        {
            min = propExt.objVRange_.fMin_;
            max = propExt.objVRange_.fMax_;
            step = propExt.objVRange_.fStep_;
        }
        break;
    }
    default:
        break;
    }
}

// get supported formats currently
void CSCamera::getFromats(STREAM_TYPE sType, std::list<std::pair<std::string, nonstd::any>>& formats) const
{
    ERROR_CODE ret;
    std::vector<StreamInfo> streamInfos;

    ret = m_cameraPtr->getStreamInfos(sType, streamInfos);
    if (ret != SUCCESS)
    {
        ROS_WARN("camera get stream info failed(%d)!\n", ret);
        return;
    }

    for (const auto& info : streamInfos)
    {
        std::pair<std::string, nonstd::any> pair_(STREAM_FORMAT_MAP.at(info.format), info.format);

        auto it = std::find_if(formats.begin(), formats.end()
            , [=](const std::pair<std::string, nonstd::any>& pair_a)
            {
                if(pair_.first == pair_a.first && pair_.second.type() == pair_a.second.type())
                    return info.format == nonstd::any_cast<STREAM_FORMAT>(pair_a.second);
                return false;
            });
        if(it == formats.end())
        {
            formats.push_back(pair_);
        }
    }
}

// get supported resolutions currently
void CSCamera::getResolutions(STREAM_TYPE sType, std::list<std::pair<std::string, nonstd::any>>& resolutions) const
{
    ERROR_CODE ret;
    std::vector<StreamInfo> streamInfos;

    STREAM_FORMAT dstFormat = (sType == STREAM_TYPE_DEPTH) ? m_depthFormat : m_rgbFormat;

    ret = m_cameraPtr->getStreamInfos(sType, streamInfos);
    if (ret != SUCCESS)
    {
        ROS_INFO("camera get stream info failed(%d)!\n", ret);
        return;
    }

    for (const auto& info : streamInfos)
    {
        if (info.format != dstFormat)
        {
            continue;
        }

        std::string resStr = std::to_string(info.width) + 'x' + std::to_string(info.height);
        cv::Size infoSize(info.width, info.height);

        std::pair<std::string, nonstd::any> pair_(resStr, infoSize);

        auto it = std::find_if(resolutions.begin(), resolutions.end()
            , [=](const std::pair<std::string, nonstd::any>& pair_a)
            {
                if(pair_.first == pair_a.first && pair_.second.type() == pair_a.second.type())
                    return infoSize == nonstd::any_cast<cv::Size>(pair_a.second);
                return false;
            });
        if(it == resolutions.end())
        {
            resolutions.push_back(pair_);
        }
    }
}

void CSCamera::getAutoExposureModes(std::list<std::pair<std::string, nonstd::any>>& list) const
{
    list.push_back({ AUTO_EXPOSURE_MODE_MAP.at(AUTO_EXPOSURE_MODE_CLOSE),          AUTO_EXPOSURE_MODE_CLOSE } );
    list.push_back({ AUTO_EXPOSURE_MODE_MAP.at(AUTO_EXPOSURE_MODE_FIX_FRAMETIME),  AUTO_EXPOSURE_MODE_FIX_FRAMETIME });
    list.push_back({ AUTO_EXPOSURE_MODE_MAP.at(AUTO_EXPOSURE_MODE_HIGH_QUALITY),   AUTO_EXPOSURE_MODE_HIGH_QUALITY });
    list.push_back({ AUTO_EXPOSURE_MODE_MAP.at(AUTO_EXPOSURE_MODE_FORE_GROUND),    AUTO_EXPOSURE_MODE_FORE_GROUND });
}

void CSCamera::getFilterTypes(std::list<std::pair<std::string, nonstd::any>>& list) const
{
    list.push_back({ FILTER_TYPE_MAP.at(FILTER_CLOSE),    FILTER_CLOSE });
    list.push_back({ FILTER_TYPE_MAP.at(FILTER_SMOOTH),   FILTER_SMOOTH });
    list.push_back({ FILTER_TYPE_MAP.at(FILTER_MEDIAN),   FILTER_MEDIAN });
    list.push_back({ FILTER_TYPE_MAP.at(FILTER_TDSMOOTH), FILTER_TDSMOOTH });
}

void CSCamera::getHdrModes(std::list<std::pair<std::string, nonstd::any>>& list) const
{
    list.push_back({ CAMERA_HDR_MAP.at(HDR_MODE_CLOSE),    HDR_MODE_CLOSE });
    list.push_back({ CAMERA_HDR_MAP.at(HDR_MODE_SHINE),    HDR_MODE_SHINE });
    list.push_back({ CAMERA_HDR_MAP.at(HDR_MODE_DARK),     HDR_MODE_DARK });
    list.push_back({ CAMERA_HDR_MAP.at(HDR_MODE_BOTH),     HDR_MODE_BOTH });
    list.push_back({ CAMERA_HDR_MAP.at(HDR_MODE_MANUAL),   HDR_MODE_MANUAL });
}

void CSCamera::getHdrLevels(std::list<std::pair<std::string, nonstd::any>>& list) const
{
    for (int i = CAMERA_HDR_LEVEL_RANGE.min; i <= CAMERA_HDR_LEVEL_RANGE.max; i++)
    {
        list.push_back({ std::to_string(i), i });
    }
}

void CSCamera::getGains(CAMERA_PARA_ID paraId, std::list<std::pair<std::string, nonstd::any>>& list)
{
    nonstd::any min, max, step;
    getCameraParaRange(paraId, min, max, step);

    int minV = nonstd::any_cast<int>(min);
    int maxV = nonstd::any_cast<int>(max);

    for (int i = minV; i <= maxV; i++)
    {
        list.push_back({ std::to_string(i), i });
    }
}

void CSCamera::setDepthFormat(STREAM_FORMAT format)
{
    if (format == m_depthFormat)
    {
        return;
    }

    // notify
    updateStreamType();

    // 1. update format
    m_depthFormat = format;
    //emit cameraParaUpdated(PARA_DEPTH_STREAM_FORMAT, (int)m_depthFormat);

    // 2. update resolution items
    std::list<std::pair<std::string, nonstd::any>> resolutions;
    getResolutions(STREAM_TYPE_DEPTH, resolutions);
    
    if(resolutions.size() <= 0)
    {
        ROS_WARN("Error, the depth resolutions is empty");
        return;
    }
    
    //emit cameraParaItemsUpdated(PARA_DEPTH_RESOLUTION);

    // 3. update resolution
    bool findRes = false;
    for (auto pair : resolutions)
    {
        if (nonstd::any_cast<cv::Size>(pair.second) == m_depthResolution)
        {
            findRes = true;
            break;
        }
    }
    
    if (!findRes)
    {
        m_depthResolution = nonstd::any_cast<cv::Size>(resolutions.begin()->second);
    }

    //emit cameraParaUpdated(PARA_DEPTH_RESOLUTION, m_depthResolution);
}

void CSCamera::setDepthResolution(cv::Size res)
{
    if (res == m_depthResolution)
    {
        return;
    }
    
    //  notify
    updateStreamType();

    // update resolution
    m_depthResolution = res;
    //emit cameraParaUpdated(PARA_DEPTH_RESOLUTION, m_depthResolution);
}

void CSCamera::setRgbFormat(STREAM_FORMAT format)
{
    if (format == m_rgbFormat)
    {
        return;
    }

    // notify
    updateStreamType();

    // 1. update format
    m_rgbFormat = format;
    //emit cameraParaUpdated(PARA_RGB_STREAM_FORMAT, (int)m_rgbFormat);

    // 2. update resolution items
    std::list<std::pair<std::string, nonstd::any>> resolutions;
    getResolutions(STREAM_TYPE_RGB, resolutions);

    if(resolutions.size() <= 0)
    {
        ROS_WARN("Error, the RGB resolutions is empty");
        return;
    }

    //emit cameraParaItemsUpdated(PARA_RGB_RESOLUTION);

    // 3. update resolution
    bool findRes = false;
    for (auto pair : resolutions)
    {
        if (nonstd::any_cast<cv::Size>(pair.second) == m_rgbResolution)
        {
            findRes = true;
        }
    }

    if (!findRes)
    {
        //m_rgbResolution = resolutions.first().second.toSize();
        m_rgbResolution = nonstd::any_cast<cv::Size>(resolutions.begin()->second);
    }

    //emit cameraParaUpdated(PARA_RGB_RESOLUTION, m_rgbResolution);
}

void CSCamera::setRgbResolution(cv::Size res)
{
    if (res == m_rgbResolution)
    {
        return;
    }

    //  notify
    updateStreamType();

    // update resolution
    m_rgbResolution = res;
    //emit cameraParaUpdated(PARA_RGB_RESOLUTION, m_rgbResolution);
}

void CSCamera::setDepthFilterValue(int value)
{
    const int min = FILTER_RANGE_MAP.at(m_filterType).min;
    const int max = FILTER_RANGE_MAP.at(m_filterType).max;

    value = (value < min) ? min : value;
    value = (value > max) ? max : value;
    
    m_filterValue = value;

    //emit cameraParaUpdated(PARA_DEPTH_FILTER, m_filterValue);
}

// update frame time before set exposure
void CSCamera::updateFrametime(float exposure)
{
    const float moreTime = 2000; // us
    const float frameTime = exposure + moreTime;

    setCameraPara(PARA_DEPTH_FRAMETIME, frameTime);
}

void CSCamera::getHdrMode(int value)
{
    if (m_hdrMode == HDR_MODE_MANUAL)
    {
        return;
    }

    HDR_MODE hdrM = (HDR_MODE)value;
    switch (hdrM)
    {
    case HDR_MODE_OFF:
        m_hdrMode = HDR_MODE_CLOSE;
        break;
    case HDR_MODE_HIGH_RELECT:
        m_hdrMode = HDR_MODE_SHINE;
        break;
    case HDR_MODE_LOW_RELECT:
        m_hdrMode = HDR_MODE_DARK;
        break;
    case HDR_MODE_ALL_RELECT:
        m_hdrMode = HDR_MODE_BOTH;
        break;
    default:
        ROS_ASSERT(false);
        break;
    }
}

int CSCamera::getAutoHdrMode(int mode)
{
    CAMERA_HDR_MODE hdrM = (CAMERA_HDR_MODE)mode;

    int autoHdrMode = HDR_MODE_OFF;
    switch (hdrM)
    {
    case HDR_MODE_CLOSE:
        autoHdrMode = HDR_MODE_OFF;
        break;
    case HDR_MODE_SHINE:
        autoHdrMode = HDR_MODE_HIGH_RELECT;
        break;
    case HDR_MODE_DARK:
        autoHdrMode = HDR_MODE_LOW_RELECT;
        break;
    case HDR_MODE_BOTH:
    case HDR_MODE_MANUAL:
        autoHdrMode = HDR_MODE_ALL_RELECT;
        break; 
    default:
        ROS_ASSERT(false);
        break;
    }

    return autoHdrMode;
}

void CSCamera::getHdrTimes(const HdrScaleSetting& settings)
{
    switch (m_hdrMode)
    {
    case HDR_MODE_SHINE:
        m_hdrTimes = settings.highReflectModeCount + 1;
        break;
    case HDR_MODE_DARK:
        m_hdrTimes = settings.lowReflectModeCount + 1;
        break;
    case HDR_MODE_CLOSE:
    case HDR_MODE_BOTH:
    case HDR_MODE_MANUAL:
        m_hdrTimes = (settings.lowReflectModeCount + settings.highReflectModeCount + 1);
        break;
    default:
        ROS_ASSERT(false);
        break;
    }
}

void CSCamera::setHdrTimes(HdrScaleSetting& settings, int times)
{
    times = (times < 2) ? 2 : times;
    times = (times > 8) ? 8 : times;

    const int scale = HDR_SCALE_DEFAULT;
    switch (m_hdrMode)
    {
    case HDR_MODE_SHINE:
        settings = { (unsigned int)(times - 1), scale, 0, scale };
        break;
    case HDR_MODE_DARK:
        settings = { 0, scale, (unsigned int)(times - 1), scale };
        break;
    case HDR_MODE_CLOSE:
    case HDR_MODE_BOTH:
    case HDR_MODE_MANUAL:
    {
        unsigned int time_high = (times - 1) / 2;
        unsigned int time_low = times - 1 - time_high;
        settings = { time_high, scale, time_low, scale };
        break;
    }
    default:
        ROS_ASSERT(false);
        break;
    }
}

CSCameraInfo CSCamera::getCameraInfo() const
{
    return m_cameraInfo;
}

int CSCamera::getCameraState() const
{
    //m_lock.lockForRead();
    m_lock.lock();
    int result = m_cameraState;
    m_lock.unlock();

    return result;
}

void CSCamera::setCameraState(CAMERA_STATE state)
{
    //m_lock.lockForWrite();
    m_lock.lock();
    m_cameraState = state;
    m_lock.unlock();

    //emit cameraStateChanged(m_cameraState);
}

void CSCamera::updateStreamType()
{
    if (getCameraState() == CAMERA_PAUSED_STREAM)
    {
        setCameraState(CAMERA_STOPPING_STREAM);
        stopStream();
    }
}

void CSCamera::onTriggerModeChanged(bool isSoftTrigger)
{
    if (isSoftTrigger)
    {
        stopStreamThread();
    }
    else 
    {
        startStreamThread();
    }
}

void CSCamera::stopStreamThread()
{
    if (m_streamThread->isRunning())
    {
        ROS_WARN("stop stream thread");
        m_streamThread->requestInterruption();
        m_streamThread->waitJoin();
    }
    else{
        ROS_WARN("already stoped stream thread");
    }
}

void CSCamera::startStreamThread()
{
    ROS_WARN("start stream thread");
    //qInfo() << "start stream thread";

    if (!m_streamThread->isRunning())
    {
        ROS_WARN("stream thread is not running, start stream thread");
        m_streamThread->start();
    }
    else{
        ROS_WARN("alredy started stream thread");
    }
}

//judge camera is network connect or not
bool CSCamera::isNetworkConnect(std::string uuid)
{
    //return uuid.contains(".");
    return uuid.find(".") != std::string::npos;
}

// sensor_msgs::CameraInfo CameraParamsToCameraInfo(int width, int height, Distort &distort, Intrinsics &intrinsics, Extrinsics &extrinsics = Extrinsics{{1,0,0, 0,1,0, 0,0,1}})
sensor_msgs::CameraInfo CameraParamsToCameraInfo(Distort &distort, Intrinsics &intrinsics, Extrinsics &extrinsics = Extrinsics{{1,0,0, 0,1,0, 0,0,1}})
{
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = "camera_link";
    camera_info.width = intrinsics.width;
    camera_info.height = intrinsics.height;

    camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    camera_info.D.resize(5, 0.0);
    camera_info.D[0] = distort.k1;
    camera_info.D[1] = distort.k2;
    camera_info.D[2] = distort.k3;
    camera_info.D[3] = distort.k4;
    camera_info.D[4] = distort.k5;

    camera_info.K.fill(0.0);
    camera_info.K[0] = intrinsics.fx;  // fx
    camera_info.K[2] = intrinsics.cx;  // cx
    camera_info.K[4] = intrinsics.fy;  // fy
    camera_info.K[5] = intrinsics.cy;  // cy
    camera_info.K[8] = 1.0;

    // camera_info.K[0] *= float(width) / intrinsics.width;  // fx
    // camera_info.K[2] *= float(width) / intrinsics.width;  // cx
    // camera_info.K[4] *= float(height) / intrinsics.height;  // fy
    // camera_info.K[5] *= float(height) / intrinsics.height;  // cy

    camera_info.R.fill(0.0);
    for (int i = 0; i < 9; i++)
        camera_info.R[i] = extrinsics.rotation[i];
    
    float R_t_mat[12];

    R_t_mat[0]  = extrinsics.rotation[0];
    R_t_mat[1]  = extrinsics.rotation[1];
    R_t_mat[2]  = extrinsics.rotation[2];
    R_t_mat[3]  = extrinsics.translation[0] / MEATER_OF_MM;
    
    R_t_mat[4]  = extrinsics.rotation[3];
    R_t_mat[5]  = extrinsics.rotation[4];
    R_t_mat[6]  = extrinsics.rotation[5];
    R_t_mat[7]  = extrinsics.translation[1] / MEATER_OF_MM;

    R_t_mat[8]  = extrinsics.rotation[6];
    R_t_mat[9]  = extrinsics.rotation[7];
    R_t_mat[10] = extrinsics.rotation[8];
    R_t_mat[11] = extrinsics.translation[2] / MEATER_OF_MM;

    camera_info.P.fill(0.0);
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            for (int p = 0; p < 3; p++) {
            camera_info.P[i * 4 + j] += camera_info.K[i * 3 + p] * R_t_mat[p * 4 + j];
            }
        }
    }

    // camera_info.R.fill(0.0);
    // camera_info.R[0] = 1.0;
    // camera_info.R[4] = 1.0;
    // camera_info.R[8] = 1.0;

    // camera_info.P[0] = camera_info.K[0];  // fx
    // camera_info.P[2] = camera_info.K[2];  // cx
    // camera_info.P[3] = 0;                 // Tx
    // camera_info.P[5] = camera_info.K[4];  // fy
    // camera_info.P[6] = camera_info.K[5];  // cy
    // camera_info.P[7] = 0;                 // Ty
    // camera_info.P[10] = 1.0;
    // camera_info.P[11] = 0.0;

    return camera_info;
}

void carmeaPreamInfoInit(void* usrData)
{
    CSCamera* camera = (CSCamera*)usrData;

    bool p_isDepthStreamSup;
    bool p_isrgbStreamSup;

    // cv::Size   p_depthResol;
    // cv::Size   p_rgbResol;
    Intrinsics p_depthIntr;
    Intrinsics p_rgbIntr;
    Distort    p_depthDistort;
    Distort    p_rgbDistort;
    Extrinsics p_extrColor;

    nonstd::any tmp;

    camera->getCameraPara(PARA_HAS_DEPTH, tmp);
    p_isDepthStreamSup = nonstd::any_cast<bool>(tmp);

    camera->getCameraPara(PARA_HAS_RGB, tmp);
    p_isrgbStreamSup = nonstd::any_cast<bool>(tmp);

    if(p_isDepthStreamSup)
    {
        // camera->getCameraPara(PARA_DEPTH_RESOLUTION, tmp);
        // p_depthResol = nonstd::any_cast<cv::Size>(tmp);

        camera->getCameraPara(PARA_DEPTH_INTRINSICS, tmp);
        p_depthIntr = nonstd::any_cast<Intrinsics>(tmp);

        camera->getCameraPara(PARA_DEPTH_DISTORT, tmp);
        p_depthDistort = nonstd::any_cast<Distort>(tmp);

        ROS_WARN("m_isDepthStreamSup CameraParam");
        // depth_camera_info_ = CameraParamsToCameraInfo(p_depthResol.width, p_depthResol.height, p_depthDistort, p_depthIntr);
        depth_camera_info_ = CameraParamsToCameraInfo(p_depthDistort, p_depthIntr);

        depth_camera_info_.header.stamp = ros::Time::now();
        depth_info.publish(depth_camera_info_);
    }

    if(p_isrgbStreamSup)
    {
        // camera->getCameraPara(PARA_RGB_RESOLUTION, tmp);
        // p_rgbResol = nonstd::any_cast<cv::Size>(tmp);

        camera->getCameraPara(PARA_RGB_INTRINSICS, tmp);
        p_rgbIntr = nonstd::any_cast<Intrinsics>(tmp);

        camera->getCameraPara(PARA_RGB_DISTORT, tmp);
        p_rgbDistort = nonstd::any_cast<Distort>(tmp);

        camera->getCameraPara(PARA_EXTRINSICS, tmp);
        p_extrColor = nonstd::any_cast<Extrinsics>(tmp);

        ROS_WARN("m_isRgbStreamSup CameraParam");
        // rgb_camera_info_ = CameraParamsToCameraInfo(p_rgbResol.width, p_rgbResol.height, p_rgbDistort, p_rgbIntr, p_extrColor);
        rgb_camera_info_ = CameraParamsToCameraInfo(p_rgbDistort, p_rgbIntr, p_extrColor);

        rgb_camera_info_.header.stamp = ros::Time::now();
        rgb_info.publish(rgb_camera_info_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "revopoint_ros");

	ros::NodeHandle nh;
    image_transport::ImageTransport it_(nh);

    // color image publisher
    pup_color = it_.advertise("/camera/color/image", 1);

    // depth image publisher
    pub_depth = it_.advertise("/camera/depth/image", 1);

    rgb_info = nh.advertise<sensor_msgs::CameraInfo> ("/camera/color/camera_info", 1);
    depth_info = nh.advertise<sensor_msgs::CameraInfo> ("/camera/depth/camera_info", 1);

    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_(nh, "/camera/color/image", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_(nh, "/camera/depth/image", 1);

    //pointcloud publisher
    pcpub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/pointcloud", 1);

    CSCamera *CS_cam = new CSCamera;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

    //message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_depth_, sub_rgb_);
	message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(80), sub_depth_, sub_rgb_);

	sync.registerCallback(boost::bind(&computePointcloud<uint16_t>, _1, _2, CS_cam));

    CS_cam->connectCamera();
    carmeaPreamInfoInit(CS_cam);
    CS_cam->startStream();
    
    ros::spin();
    
    ROS_WARN("CS_cam.stopStream()");
    CS_cam->stopStream();

	return 0;
}