/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
//#define GL_GLEXT_PROTOTYPES 1
//#define GL3_PROTOTYPES 1
//#include <GL/glew.h>

#include "QGLViewer/qglviewer.h"
#include <vector>
#include "boost/thread.hpp"
#include "qevent.h"
#include <string>     // std::string, std::to_string
#include "lsd_slam_viewer/keyframeMsg.h"
#include "lsd_slam_viewer/keyframeGraphMsg.h"

#include "QGLViewer/keyFrameInterpolator.h"

class QApplication;

class KeyFrameGraphDisplay;

class CameraDisplay;

class KeyFrameDisplay;

#include "settings.h"


class AnimationObject {
public:
    double time;
    double duration;

    // settings
    float scaledTH;
    float absTH;
    int neighb;
    int sparsity;
    bool showLoopClosures;
    bool showKeyframes;
    bool showCurrentCam;

    // frame
    qglviewer::Frame frame;

    // whether is a KF or only settings change
    bool isSettings;

    bool isFix;

    AnimationObject(bool isSettings, double time, double duration, qglviewer::Frame f = qglviewer::Frame()) {
        this->time = time;
        this->duration = duration;

        scaledTH = scaledDepthVarTH;
        absTH = absDepthVarTH;
        neighb = minNearSupport;
        showKeyframes = showKFCameras;
        showLoopClosures = showConstraints;
        showCurrentCam = showCurrentCamera;
        sparsity = sparsifyFactor;

        this->isSettings = isSettings;

        frame = f;

        isFix = false;
    }

    AnimationObject(std::string s) {
        int isSettings_i;
        int showLoopClosures_i;
        int showKeyframes_i;
        int showCurrentCam_i;
        int isFix_i;


        qglviewer::Quaternion orient;


        double x, y, z;

        if (17 != sscanf(
                s.c_str(),
                "Animation: %s at %s (dur %s) S: %s %s %s %s %s %s %s Frame: %s %s %s %s %s %s %s %s\n",
                std::to_string(&isSettings_i),
                std::to_string(&time),
                std::to_string(&duration),
                std::to_string(&scaledTH),
                std::to_string(&absTH),
                std::to_string(&showLoopClosures_i),
                std::to_string(&showKeyframes_i),
                std::to_string(&showCurrentCam_i),
                std::to_string(&sparsity),
                std::to_string(&neighb),
                std::to_string(&(orient[0])),
                std::to_string(&(orient[1])),
                std::to_string(&(orient[2])),
                std::to_string(&(orient[3])),
                std::to_string(&x),
                std::to_string(&y),
                std::to_string(&z),
                std::to_string(&isFix_i)
        ))
            printf("error parsing: %s\n",
                   s.c_str()
            );

        isSettings = isSettings_i;
        showLoopClosures = showLoopClosures_i;
        showKeyframes = showKeyframes_i;
        showCurrentCam = showCurrentCam_i;
        isFix = isFix_i;


        frame = qglviewer::Frame(qglviewer::Vec(0, 0, 0), orient);
        frame.setPosition(x, y, z);

        printf("read: %s\n",
               toString().c_str()
        );
    }

    bool operator<(const AnimationObject &other) const {
        return (time < other.time);
    }

    std::string toString() {
        char buf[1000];

        int isSettings_i = isSettings;
        int showLoopClosures_i = showLoopClosures;
        int showKeyframes_i = showKeyframes;
        int showCurrentCam_i = showCurrentCam;
        int isFix_i = isFix;

        double x, y, z;
        frame.getPosition(x, y, z);

        snprintf(buf, 1000, "Animation: %s at %s (dur %s) S: %s %s %s %s %s %s %s Frame: %s %s %s %s %s %s %s %s",
                 std::to_string(isSettings_i),
                 std::to_string(time),
                 std::to_string(duration),
                 std::to_string(scaledTH),
                 std::to_string(absTH),
                 std::to_string(showLoopClosures_i),
                 std::to_string(showKeyframes_i),
                 std::to_string(showCurrentCam_i),
                 std::to_string(sparsity),
                 std::to_string(neighb),
                 std::to_string(frame.orientation()[0]),
                 std::to_string(frame.orientation()[1]),
                 std::to_string(frame.orientation()[2]),
                 std::to_string(frame.orientation()[3]),
                 std::to_string(x),
                 std::to_string(y),
                 std::to_string(z),
                 std::to_string(isFix_i)
        );

        return buf;
    }
};


class PointCloudViewer : public QGLViewer {
public:
    PointCloudViewer();

    ~PointCloudViewer();


    void reset();

    void addFrameMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);

    void addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg);


protected :
    virtual void draw();

    virtual void init();

    virtual void keyPressEvent(QKeyEvent *e);

    virtual void keyReleaseEvent(QKeyEvent *e);

    virtual QString helpString() const;

//	virtual void drawText(int x, int y, const QString & text, const QFont & fnt) {printf(text.toStdString().c_str());};


private:

    // displays kf-graph
    KeyFrameGraphDisplay *graphDisplay;

    // displays only current keyframe (which is not yet in the graph).
    KeyFrameDisplay *currentCamDisplay;


    // meddle mutex
    boost::mutex meddleMutex;


    void setToVideoSize();

    bool resetRequested;

    // for saving stuff
    std::string save_folder;
    double localMsBetweenSaves;
    double simMsBetweenSaves;
    double lastSaveTime;
    double lastCamTime;
    int lastCamID;


    double lastLocalSaveTime;
    double lastRealSaveTime;


    // for keyframe interpolation
    int KFLastPCSeq;
    int KFcurrent;
    double KFautoPlayIdx[10];
    bool KFexists[10];
    double lastAutoplayCheckedSaveTime;

    // for display settings autoplay
    std::vector <AnimationObject> animationList;
    qglviewer::KeyFrameInterpolator *kfInt;
    bool customAnimationEnabled;

    bool animationPlaybackEnabled;
    double animationPlaybackTime;
    int animationPlaybackID;


    double lastAnimTime;


    void remakeAnimation();
};


