/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/
#pragma once

#include "stdafx.h"
#include "calibrationSettings.h"

#define CID_CAMERA_CALIBRATION  "camera_calibration.filter.demo.aadc.cid"

class cCameraCalibrationWidget;

/*! the main class for the camera calibration filter. */
class cCameraCalibrationFilter : public QObject, virtual public cQtUIFilter
{
    Q_OBJECT

signals:
    /*! sends the image of the current media sample to the gui
    * \param newImage the image
    */
    void newImage(const QImage& newImage);

    /*! send the state to the gui
    * \param state the state with the enum
    */
    void sendState(int state);

    /*!
     * Sends a message.
     *
     * \param   message The message.
     */
    void sendMessage(cString message);

public slots:

    /*! slot for starting the calibration */
    void OnStartCalibration();

    /*! slot for starting the calibration */
    void OnStartCalibrationFisheye();

    /*! Executes the cancel action. */
    void OnCancel();

    /*! slot for saving the file
    * \param qFilename the filename including path where to save
    */
    void OnSaveAs(QString qFilename);

public:
    ADTF_CLASS_ID_NAME(cCameraCalibrationFilter, CID_CAMERA_CALIBRATION, "Camera Calibration");
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::ui::ant::IQtXSystem),
                            REQUIRE_INTERFACE(adtf::services::IReferenceClock));

private:

    /*! Pins: */
    cPinReader     m_oPinInputVideo;

    /*! Stream Formats. */
    adtf::streaming::tStreamImageFormat m_sInputFormat;

    /*! ui. */
    cCameraCalibrationWidget* m_pWidget;

    /*! the pointer which holds the image data shown in the gui. */
    Mat inputMat;

    /*! the timestamp of the last sample to measure the delay*/
    clock_t m_prevTimestamp;

    /*! the image points (i.e. results) of the measurements */
    vector<vector<Point2f> > m_imagePoints;

    /*! the current state of filter */
    tInt8 m_calibrationState;

    /*! the size of the frames*/
    Size m_matrixSize;

    /*! the camera matrix of the calibration */
    Mat m_cameraMatrix;

    /*! the distortion coefficients of the camera*/
    Mat m_distCoeffs;
    
    /*! The calib patter */
    property_variable<tInt>     m_calibPatter              = 1;
    /*! The calib debug output to console */
    property_variable<tBool>    m_calibDebugOutputToConsole;
    /*! Width of the calib */
    property_variable<tInt>     m_calibWidth               = 9;
    /*! Height of the calib */
    property_variable<tInt>     m_calibHeight              = 6;
    /*! The calib delay */
    property_variable<tFloat32> m_calibDelay               = 0.5f;
    /*! The calib datasets to use */
    property_variable<tUInt64>   m_calibDatasetsToUse       = 10;
    /*! Size of the calib square */
    property_variable<tFloat32> m_calibSquareSize          = 0.025f;
    /*! The calib aspect ration */
    property_variable<tFloat32> m_calib_AspectRation       = 1.0f; 

    /*! if we use fisheye model for calibration*/
    calibrationSettings m_calibrationSettings;

    /*! indicates wheter information is printed to the console or not */
    tBool m_bDebugModeEnabled;

    /*!
     * function process the video data.
     *
     * \param [in,out]  inputMat    the new media sample to be processed.
     *
     * \return  Returns a standard result code.
     */
    tResult ProcessVideo(Mat& inputMat);

    /*!
     * returns the current streamtime.
     *
     * \return  timestamp returns stream time in microseconds.
     */
    tTimeStamp GetTime();

    /*! calls the calculation of the matrixes and saving them to the given file
    (original Source is at opencv/samples/cpp/calibration.cpp)
    * \param imageSize refer to opencv/samples/cpp/calibration.cpp
    * \param cameraMatrix refer to opencv/samples/cpp/calibration.cpp
    * \param distCoeffs refer to opencv/samples/cpp/calibration.cpp
    * \param imagePoints refer to opencv/samples/cpp/calibration.cpp
    * \result Returns a standard result code.
    */
    tBool runCalibrationAndSave(Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                                vector<vector<Point2f>> imagePoints);

    /*! does the calculation
    (original Source is at opencv/samples/cpp/calibration.cpp)
    * \param imageSize refer to opencv/samples/cpp/calibration.cpp
    * \param cameraMatrix refer to opencv/samples/cpp/calibration.cpp
    * \param distCoeffs refer to opencv/samples/cpp/calibration.cpp
    * \param imagePoints refer to opencv/samples/cpp/calibration.cpp
    * \param rvecs refer to opencv/samples/cpp/calibration.cpp
    * \param tvecs refer to opencv/samples/cpp/calibration.cpp
    * \param reprojErrs refer to opencv/samples/cpp/calibration.cpp
    * \param totalAvgErr refer to opencv/samples/cpp/calibration.cpp
    * \result Returns a standard result code.
    */
    tBool runCalibration(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                         vector<vector<Point2f>> imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                         vector<float>& reprojErrs, double& totalAvgErr);

    /*! saves the camera params
    (original Source is at opencv/samples/cpp/calibration.cpp)
    * \param imageSize refer to opencv/samples/cpp/calibration.cpp
    * \param cameraMatrix refer to opencv/samples/cpp/calibration.cpp
    * \param distCoeffs refer to opencv/samples/cpp/calibration.cpp
    * \param rvecs refer to opencv/samples/cpp/calibration.cpp
    * \param tvecs refer to opencv/samples/cpp/calibration.cpp
    * \param reprojErrs refer to opencv/samples/cpp/calibration.cpp
    * \param imagePoints refer to opencv/samples/cpp/calibration.cpp
    * \param totalAvgErr refer to opencv/samples/cpp/calibration.cpp
    * \result Returns nothing
    */
    tVoid saveCameraParams(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                           const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                           double totalAvgErr);

    /*! computes the reprojection errors
    (original Source is at opencv/samples/cpp/calibration.cpp)
    * \param objectPoints refer to opencv/samples/cpp/calibration.cpp
    * \param imagePoints refer to opencv/samples/cpp/calibration.cpp
    * \param rvecs refer to opencv/samples/cpp/calibration.cpp
    * \param tvecs refer to opencv/samples/cpp/calibration.cpp
    * \param cameraMatrix refer to opencv/samples/cpp/calibration.cpp
    * \param distCoeffs refer to opencv/samples/cpp/calibration.cpp
    * \param perViewErrors refer to opencv/samples/cpp/calibration.cpp
    * \param fisheye refer to opencv/samples/cpp/calibration.cpp
    * \result reprojection error
    */
    static double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors, bool fisheye);

    /*! calculates the chessboard corners
    (original Source is at opencv/samples/cpp/calibration.cpp)
    * \param boardSize size of board
    * \param squareSize size of square
    * \param corners vector with corners
    * \param patternType type of used pattern
    */
    static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners,
                                      calibrationSettings::Pattern patternType = calibrationSettings::CHESSBOARD);

    /*! this function resets all the calibrations results to start new one
    * \result Returns a standard result code.
    */
    tResult resetCalibrationResults();

    /*! The mutex */
    std::mutex m_oMutex;
public:
    /*! Default constructor. */
    cCameraCalibrationFilter();
    /*! Destructor. */
    virtual ~cCameraCalibrationFilter();

protected: // Implement cBaseQtFilter
    QWidget * CreateView() override;
    tVoid    ReleaseView() override;
    tResult OnIdle() override;
    tResult  OnTimer() override;
    tResult  Init(tInitStage eStage) override;
};
