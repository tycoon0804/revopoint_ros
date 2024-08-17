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

#ifndef _CS_CSCAMERA_H
#define _CS_CSCAMERA_H

#include "../include/nonstd/any.hpp"

#include <string>
#include <list> 
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

//#include <QObject>
//#include <QVector>
//#include <QRectF>
//#include <QThread>
//#include <QMetaEnum>
//#include <QReadWriteLock>

#include <hpp/System.hpp>
#include <hpp/Camera.hpp>

//#include "icscamera.h"
#include "../include/cstypes.h"
#include "../include/cameraparaid.h"

#define GET_FRAME_TIME_OUT 10         //ms

namespace cs {

using namespace parameter;
class CSCamera// : public ICSCamera
{
    //Q_OBJECT
public:
    CSCamera();
    ~CSCamera();
    static int setCameraChangeCallback(CameraChangeCallback callback, void* userData);
    static int setCameraAlarmCallback(CameraAlarmCallback callback, void* userData);
    static void setSdkLogPath(std::string path);
    static void queryCameras(std::vector<CameraInfo>& cameras);

    void getCameraPara(CAMERA_PARA_ID paraId, nonstd::any& value);// override;
    void setCameraPara(CAMERA_PARA_ID paraId, nonstd::any value);// override;
    void getCameraParaRange(CAMERA_PARA_ID paraId, nonstd::any& min, nonstd::any& max, nonstd::any& step); //override;
    void getCameraParaItems(CAMERA_PARA_ID paraId, std::list<std::pair<std::string, nonstd::any>>& list); //override;

    bool connectCamera(CameraInfo info);
    bool connectCamera();
    bool disconnectCamera(); //override;
    bool reconnectCamera(); //override;
    bool restartCamera(); //override;

    bool startStream(); //override;
    bool stopStream(); //override;
    bool restartStream(); //override;
    bool pauseStream(); //override;
    bool resumeStream(); //override;
    bool softTrigger(); //override;

    int onGetFrame(FrameData& frameData, int timeout = GET_FRAME_TIME_OUT);
    int doGetFrame(std::vector<StreamData>& streamDatas, int timeout = GET_FRAME_TIME_OUT);

    bool onProcessFrame(STREAM_DATA_TYPE streamDataType, const IFramePtr& frame, std::vector<StreamData>& streamDatas);

    //void setCameraThread(std::thread&& thread);
    void setCameraThread(std::thread* thread);

    CSCameraInfo getCameraInfo() const; //override;
    int getCameraState() const; //override;

    // Intrinsics m_depthIntrinsics;
    // Intrinsics m_rgbIntrinsics;
    // Extrinsics m_extrinsics;

    // float m_depthScale;
    mutable std::mutex m_lock;
private:
    bool startRgbStream();
    bool startDepthStream();
    bool stopDepthStream();
    bool stopRgbStream();
    void doDisconnectCamera();
private:
    void setCameraState(CAMERA_STATE state);
    //sensor_msgs::CameraInfo OBCameraNode::OBCameraParamsToCameraInfo(Distort &distort, Intrinsics &intrinsics);
    void initDefaultStreamInfo();
    void initCameraInfo();
    StreamInfo getDepthStreamInfo();
    StreamInfo getRgbStreamInfo();

    void getFromats(STREAM_TYPE sType, std::list<std::pair<std::string, nonstd::any>>&) const;
    void getResolutions(STREAM_TYPE sType, std::list<std::pair<std::string, nonstd::any>>&) const;
    void getAutoExposureModes(std::list<std::pair<std::string, nonstd::any>>&) const;
    void getFilterTypes(std::list<std::pair<std::string, nonstd::any>>&) const;
    void getHdrModes(std::list<std::pair<std::string, nonstd::any>>&) const;
    void getHdrLevels(std::list<std::pair<std::string, nonstd::any>>&) const;
    void getGains(CAMERA_PARA_ID paraId, std::list<std::pair<std::string, nonstd::any>>&);

    void setDepthFormat(STREAM_FORMAT format);
    void setDepthResolution(cv::Size res);
    void setRgbFormat(STREAM_FORMAT format);
    void setRgbResolution(cv::Size res);
    void getHdrMode(int value);
    int getAutoHdrMode(int mode);
    void getHdrTimes(const HdrScaleSetting& settings);
    void setHdrTimes(HdrScaleSetting& settings, int times);
    void updateFrametime(float exposure);

    void setDepthFilterValue(int value);
    void updateStreamType();
    void onTriggerModeChanged(bool isSoftTrigger);

    void stopStreamThread();
    void startStreamThread();

    bool isNetworkConnect(std::string uuid);
    void restoreExposureGain();
//signals:
    //void updateParaSignal(int paraId);
private: //private slots:
    void onParaLinkResponse(CAMERA_PARA_ID paraId, const nonstd::any& value);
    void onParaUpdated(int paraId);
    void onParaUpdatedDelay(CAMERA_PARA_ID paraId, int delayMS);
    void onStreamStarted();
//private:
    void getUserParaPrivate(CAMERA_PARA_ID paraId, nonstd::any& value);
    void setUserParaPrivate(CAMERA_PARA_ID paraId, nonstd::any value);
    void getUserParaRangePrivate(CAMERA_PARA_ID paraId, nonstd::any& min, nonstd::any& max, nonstd::any& step);

    //void getPropertyPrivate(CAMERA_PARA_ID paraId, nonstd::any& value);
    void getPropertyPrivate(CAMERA_PARA_ID paraId, nonstd::any& value);
    void setPropertyPrivate(CAMERA_PARA_ID paraId, nonstd::any value);

    void getPropertyRangePrivate(CAMERA_PARA_ID paraId, nonstd::any& min, nonstd::any& max, nonstd::any& step);

    void getExtensionPropertyPrivate(CAMERA_PARA_ID paraId, nonstd::any& value);
    void setExtensionPropertyPrivate(CAMERA_PARA_ID paraId, nonstd::any value);
    void getExtensionPropertyRangePrivate(CAMERA_PARA_ID paraId, nonstd::any& min, nonstd::any& max, nonstd::any& step);

private:
    //class StreamThread : public QThread
    class StreamThread
    {
    public:
        StreamThread(CSCamera& camera);
        ~StreamThread();
        void run() const;
        void start();
        bool isRunning();
        void requestInterruption();
        void waitJoin();
    private:
        CSCamera& m_camera;
        std::mutex m_Mutex;
        std::unique_lock<std::mutex> lk;
        std::condition_variable cv_;
        std::atomic<bool> m_isInterruptionRequested;
        std::thread* m_StreamThread;
    };

private:
    static const std::map<int, const char*> AUTO_EXPOSURE_MODE_MAP;
    static const std::map<int, const char*> FILTER_TYPE_MAP;
    static const std::map<CAMERA_HDR_MODE, const char*> CAMERA_HDR_MAP;
    //static QMetaEnum metaEnum;
private:
    CAMERA_STATE m_cameraState;
    ICameraPtr m_cameraPtr;
    CSCameraInfo m_cameraInfo;

    STREAM_FORMAT m_depthFormat;
    STREAM_FORMAT m_rgbFormat;
    cv::Size m_depthResolution;
    cv::Size m_rgbResolution;
    TRIGGER_MODE m_triggerMode;

    int m_filterValue;
    int m_filterType;
    bool m_fillHole;

    bool m_hasIrStream;
    bool m_hasDepthStream;

    bool m_isRgbStreamSup;
    bool m_isDepthStreamSup;

    //hdr
    CAMERA_HDR_MODE m_hdrMode;
    int m_hdrTimes;
    HdrExposureSetting m_manualHdrSetting;
    //for restoring exposure and gain when cloing HDR
    int m_cachedDepthExposure;
    int m_cachedDepthGain;

    Intrinsics m_depthIntrinsics;
    Intrinsics m_rgbIntrinsics;
    Extrinsics m_extrinsics;
    Distort    m_depthDistort;
    Distort    m_rgbDistort;
    float m_depthScale;

    StreamThread* m_streamThread;
    //friend StreamThread;

    //QThread* m_cameraThread;
    std::thread* m_cameraThread;
    //mutable QReadWriteLock m_lock;
    //mutable std::mutex m_lock;
};
}

#endif // _CS_CSCAMERA_H
