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

#include "CameraCalibrationFilter.h"
#include "ADTF3_helper.h"
#include "ADTF3_OpenCV_helper.h"

ADTF_PLUGIN("Camera Calibration Plugin", cCameraCalibrationFilter)


cCameraCalibrationFilter::cCameraCalibrationFilter() : m_pWidget(nullptr)
{
    RegisterPropertyVariable("Calibration Pattern"         ,  m_calibPatter                  );
    RegisterPropertyVariable("Debug Output to Console"     ,  m_calibDebugOutputToConsole    );
    RegisterPropertyVariable("Width [number of squares]"   ,  m_calibWidth                   );
    RegisterPropertyVariable("Height [number of squares]"  ,  m_calibHeight                  );
    RegisterPropertyVariable("Delay [s]"                   ,  m_calibDelay                   );
    RegisterPropertyVariable("Number of Datasets to use"   ,  m_calibDatasetsToUse           );
    RegisterPropertyVariable("Square Size [m]"             ,  m_calibSquareSize              );
    RegisterPropertyVariable("Aspect Ratio"                ,  m_calib_AspectRation           );

    //create and set inital input format type
    m_sInputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sInputFormat);

    //Register input pin
    create_pin(*this, m_oPinInputVideo, "video_rgb", pType);

    //register callback for type changes
    m_oPinInputVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oPinInputVideo, m_sInputFormat, *pType.Get());
    });
}


cCameraCalibrationFilter::~cCameraCalibrationFilter()
{

}

QWidget* cCameraCalibrationFilter::CreateView()
{
    // use single UI File in background
    m_pWidget = new cCameraCalibrationWidget(nullptr);

    connect(this, SIGNAL(newImage(const QImage &)), m_pWidget, SLOT(OnNewImage(const QImage &)));
    connect(m_pWidget->m_btStart, SIGNAL(clicked()), this, SLOT(OnStartCalibration()));
    connect(m_pWidget->m_btStartFisheye, SIGNAL(clicked()), this, SLOT(OnStartCalibrationFisheye()));
    connect(m_pWidget->m_btCancel, SIGNAL(clicked()), this, SLOT(OnCancel()));
    connect(m_pWidget, SIGNAL(SendSaveAs(QString)), this, SLOT(OnSaveAs(QString)));
    connect(this, SIGNAL(sendState(int)), m_pWidget, SLOT(OnSetState(int)));
    connect(this, SIGNAL(sendMessage(cString)), m_pWidget, SLOT(OnSendMessage(cString)));

    return m_pWidget;
}


tVoid cCameraCalibrationFilter::ReleaseView()
{
    delete m_pWidget;
    m_pWidget = nullptr;
}

tResult cCameraCalibrationFilter::OnIdle()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    RETURN_NOERROR;
}

tResult cCameraCalibrationFilter::OnTimer()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    object_ptr<const ISample> pReadSample;

    if(IS_OK(m_oPinInputVideo.GetLastSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat m_inputImage = Mat(cv::Size(m_sInputFormat.m_ui32Width, m_sInputFormat.m_ui32Height),
                                   CV_8UC3, (uchar*) pReadBuffer->GetPtr());

            //Do the image processing and copy to destination image buffer
            ProcessVideo(m_inputImage);
        }
        pReadBuffer->Unlock();
    }

    RETURN_NOERROR;
}

tResult cCameraCalibrationFilter::Init(tInitStage eStage)
{   
    // set the member variables from the properties
    // debug mode or not
    m_bDebugModeEnabled = m_calibDebugOutputToConsole;
    // get calibration pattern
    if (m_calibPatter == 1) m_calibrationSettings.calibrationPattern = calibrationSettings::CHESSBOARD;
    // get width and height of board
    m_calibrationSettings.boardSize = Size(m_calibWidth, m_calibHeight);

    // set the member variables for the calibration (see header)
    m_calibrationSettings.delay = m_calibDelay * 1e6;
    m_calibrationSettings.nrFrames = m_calibDatasetsToUse;
    m_calibrationSettings.nrFramesDataAq = m_calibrationSettings.nrFrames * 5;
    m_calibrationSettings.squareSize = m_calibSquareSize;
    m_calibrationSettings.aspectRatio = m_calib_AspectRation;
    m_calibrationSettings.writeExtrinsics = true;
    m_calibrationSettings.writePoints = false;
    m_calibrationSettings.calibZeroTangentDist = true;
    m_calibrationSettings.calibFixPrincipalPoint = true;
    m_calibrationSettings.flag = 0;
    // set the state and other necessary variables
    m_prevTimestamp = 0;
    m_imagePoints.clear();
    m_calibrationState = WAITING;
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
    RETURN_NOERROR;
}

tResult cCameraCalibrationFilter::ProcessVideo(Mat& inputMat)
{
    //copy to new mat and convert to grayscale otherwise the image in original media sample is modified
    Mat frame_gray;
    cvtColor(inputMat, frame_gray, COLOR_BGR2GRAY);

    //variables for detection
    tBool bFoundPattern = tFalse;
    vector<Point2f> pointbuf;

    // is not calibrated and is not waiting
    if (m_calibrationState == CAPTURING)
    {
     
        //m_countAttemptsSearchChessboard++;
        //LOG_INFO(cString::Format("Attempt %i/%i", m_countAttemptsSearchChessboard, m_maxCountAttemptsSearchChessboard));


        int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

        if (!m_calibrationSettings.useFisheye)
        {
            // fast check erroneously fails with high distortions like fisheye
            chessBoardFlags |= CALIB_CB_FAST_CHECK;
        }

        //find pattern in input image, result is written to pointbuf
        switch (m_calibrationSettings.calibrationPattern)
        {
            case calibrationSettings::CHESSBOARD:
                bFoundPattern = findChessboardCorners(inputMat, m_calibrationSettings.boardSize, pointbuf,
                                                      chessBoardFlags);
                break;
            default:
                LOG_ERROR("Unknown Calibration Pattern chosen");
                break;
        }

        // improve the found corners' coordinate accuracy
        if (bFoundPattern)
        {
            //m_countAttemptsSearchChessboard = 0;

            if (m_calibrationSettings.calibrationPattern == calibrationSettings::CHESSBOARD)
            {
                cornerSubPix(frame_gray, pointbuf, Size(11, 11),
                             Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 300, 0.1));
            }

            //if the delay time is passed and the pattern is found create a valid calibration dataset in m_imagePoints
            if ((abs(m_prevTimestamp - GetTime()) > m_calibrationSettings.delay))
            {
                m_imagePoints.push_back(pointbuf);
                m_prevTimestamp = GetTime();
                emit sendMessage(cString::Format("Processed %d of %d needed datasets", m_imagePoints.size(), m_calibrationSettings.nrFramesDataAq));
                LOG_INFO(cString::Format("Processed %d of %d needed datasets", m_imagePoints.size(), m_calibrationSettings.nrFramesDataAq));
            }
            else
            {
                emit sendMessage(cString::Format("Delay time was not passed"));
                LOG_INFO(cString::Format("Delay time was not passed"));
            }

            //draw the corners in the gray image
            drawChessboardCorners(frame_gray, m_calibrationSettings.boardSize, Mat(pointbuf), bFoundPattern);
        }
        //if more image points than the defined numbers are found switch to state CALIBRATED
        if (m_imagePoints.size() >= unsigned(m_calibrationSettings.nrFramesDataAq))
        {
            m_matrixSize = frame_gray.size();
            m_calibrationState = CAPTURING_FINISHED;
            emit sendState(m_calibrationState);
        }

    }

    //create an qimage from the data to print to qt gui
    QImage image(frame_gray.data, frame_gray.cols, frame_gray.rows, static_cast<int>(frame_gray.step), QImage::Format_Indexed8);

    // for a grayscale image there is no suitable definition in qt so we have to create our own color table
    QVector<QRgb> grayscaleTable;
    for (int i = 0; i < 256; i++) grayscaleTable.push_back(qRgb(i, i, i));
    image.setColorTable(grayscaleTable);

    //send the image to the gui
    emit newImage(image.scaled(320, 240, Qt::KeepAspectRatio));

    RETURN_NOERROR;
}

tBool cCameraCalibrationFilter::runCalibrationAndSave(Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                                                      vector<vector<Point2f>> imagePoints)
{
    // for more details and comments look to original Source at opencv/samples/cpp/calibration.cpp
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    tFloat64 totalAvgErr = 0;
    bool ok = tFalse;

    try
    {
        ok = runCalibration(imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                            totalAvgErr);
    }
    catch (cv::Exception& e)
    {
        LOG_ERROR(cString::Format("CV exception caught: %s", e.what()));
    }

    if (ok)
        LOG_INFO(cString::Format("Calibration succeeded, avg reprojection error = %.2f", totalAvgErr));
    else
        LOG_INFO(cString::Format("Calibration failed, avg reprojection error = %.2f", totalAvgErr));

    if (ok)
        saveCameraParams(imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                         totalAvgErr);
    return ok;
}

tBool cCameraCalibrationFilter::runCalibration(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                                               vector<vector<Point2f>> imagePoints, vector<Mat>& rvecs,
                                               vector<Mat>& tvecs,
                                               vector<float>& reprojErrs, double& totalAvgErr)
{


    vector<vector<Point3f> > objectPoints(1);
    calcChessboardCorners(m_calibrationSettings.boardSize, m_calibrationSettings.squareSize, objectPoints[0], m_calibrationSettings.calibrationPattern);

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    bool ok = false;
    bool rerunCalibration = true;
    while (rerunCalibration)
    {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        //! [fixed_aspect]
        if (m_calibrationSettings.flag & CALIB_FIX_ASPECT_RATIO)
            cameraMatrix.at<double>(0, 0) = m_calibrationSettings.aspectRatio;
        //! [fixed_aspect]

        if (m_calibrationSettings.useFisheye)
        {
            distCoeffs = Mat::zeros(4, 1, CV_64F);

            Mat _rvecs, _tvecs;
            rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
                                     _tvecs, m_calibrationSettings.flag);

            rvecs.reserve(_rvecs.rows);
            tvecs.reserve(_tvecs.rows);
            for (int i = 0; i < int(objectPoints.size()); i++)
            {
                rvecs.push_back(_rvecs.row(i));
                tvecs.push_back(_tvecs.row(i));
            }
        }
        else
        {
            distCoeffs = Mat::zeros(8, 1, CV_64F);

            rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
                                  m_calibrationSettings.flag);
        }

        LOG_INFO(cString::Format("Re-projection error reported by calibrateCamera: %.2f", rms));;

        ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

        totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                                distCoeffs, reprojErrs, m_calibrationSettings.useFisheye);

        rerunCalibration = false;
        while (objectPoints.size() > (size_t) m_calibrationSettings.nrFrames)
        {
            int pos = std::max_element(reprojErrs.begin(), reprojErrs.end()) - reprojErrs.begin();

            objectPoints.erase(objectPoints.begin() + pos);
            imagePoints.erase(imagePoints.begin() + pos);
            reprojErrs.erase(reprojErrs.begin() + pos);

            rerunCalibration = true;
        }
    }

    return ok;
}


void cCameraCalibrationFilter::saveCameraParams(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                                          const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                          const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                                          double totalAvgErr)
{
    FileStorage fs(m_calibrationSettings.outputFileName, FileStorage::WRITE);

    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty())
        fs << "nr_of_frames" << (int) std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << m_calibrationSettings.boardSize.width;
    fs << "board_height" << m_calibrationSettings.boardSize.height;
    fs << "square_size" << m_calibrationSettings.squareSize;

    if (m_calibrationSettings.flag & CALIB_FIX_ASPECT_RATIO)
        fs << "fix_aspect_ratio" << m_calibrationSettings.aspectRatio;

    if (m_calibrationSettings.flag)
    {
        std::stringstream flagsStringStream;
        if (m_calibrationSettings.useFisheye)
        {
            flagsStringStream << "flags:"
                << (m_calibrationSettings.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
                << (m_calibrationSettings.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
                << (m_calibrationSettings.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
                << (m_calibrationSettings.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
                << (m_calibrationSettings.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
                << (m_calibrationSettings.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        }
        else
        {
            flagsStringStream << "flags:"
                << (m_calibrationSettings.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                << (m_calibrationSettings.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                << (m_calibrationSettings.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                << (m_calibrationSettings.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                << (m_calibrationSettings.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
                << (m_calibrationSettings.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
                << (m_calibrationSettings.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
                << (m_calibrationSettings.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
                << (m_calibrationSettings.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
        }
        fs.writeComment(flagsStringStream.str());
    }

    fs << "flags" << m_calibrationSettings.flag;

    fs << "fisheye_model" << m_calibrationSettings.useFisheye;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (m_calibrationSettings.writeExtrinsics && !reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if (m_calibrationSettings.writeExtrinsics && !rvecs.empty() && !tvecs.empty())
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int) rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
        bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
        bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

        for (size_t i = 0; i < rvecs.size(); i++)
        {
            Mat r = bigmat(Range(int(i), int(i + 1)), Range(0, 3));
            Mat t = bigmat(Range(int(i), int(i + 1)), Range(3, 6));

            if (needReshapeR)
                rvecs[i].reshape(1, 1).copyTo(r);
            else
            {
                //*.t() is MatExpr (not Mat) so we can use assignment operator
                CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                r = rvecs[i].t();
            }

            if (needReshapeT)
                tvecs[i].reshape(1, 1).copyTo(t);
            else
            {
                CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                t = tvecs[i].t();
            }
        }
        fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
        fs << "extrinsic_parameters" << bigmat;
    }

    if (m_calibrationSettings.writePoints && !imagePoints.empty())
    {
        Mat imagePtMat((int) imagePoints.size(), (int) imagePoints[0].size(), CV_32FC2);
        for (size_t i = 0; i < imagePoints.size(); i++)
        {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

double cCameraCalibrationFilter::computeReprojectionErrors(
    const vector<vector<Point3f> >& objectPoints,
    const vector<vector<Point2f> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (size_t i = 0; i < objectPoints.size(); ++i)
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        if (n != 0)
            perViewErrors[i] = (float) std::sqrt(err*err / n);
        totalErr += err * err;
        totalPoints += n;
    }
    if (totalPoints != 0)
    {
        return std::sqrt(totalErr / totalPoints);
    }
    return 0;
}

void cCameraCalibrationFilter::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, calibrationSettings::Pattern patternType)
{
    corners.clear();

    switch (patternType)
    {
        case calibrationSettings::CHESSBOARD:
        case calibrationSettings::CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; ++i)
                for (int j = 0; j < boardSize.width; ++j)
                    corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
            break;

        case calibrationSettings::ASYMMETRIC_CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.push_back(Point3f((2 * j + i % 2)*squareSize, i*squareSize, 0));
            break;
        default:
            break;
    }
}

tResult cCameraCalibrationFilter::resetCalibrationResults()
{
    m_matrixSize = cv::Size(0, 0);
    m_imagePoints.clear();
    m_prevTimestamp = 0;
    m_cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (m_calibrationSettings.flag & CALIB_FIX_ASPECT_RATIO)
    {
        m_cameraMatrix.at<double>(0, 0) = m_calibrationSettings.aspectRatio;
    }
    //! [fixed_aspect]
    if (m_calibrationSettings.useFisheye)
    {
        m_distCoeffs = Mat::zeros(4, 1, CV_64F);
    }
    else
    {
        m_distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    RETURN_NOERROR;
}

tTimeStamp cCameraCalibrationFilter::GetTime()
{
    return (_clock != NULL) ? _clock->GetTime() : cSystem::GetTime();
}

void cCameraCalibrationFilter::OnStartCalibration()
{
    // switch state to CAPTURING
    resetCalibrationResults();
    m_calibrationState = CAPTURING;
    m_calibrationSettings.flag = 0;
    m_calibrationSettings.flag = CV_CALIB_FIX_ASPECT_RATIO;
    m_calibrationSettings.useFisheye = tFalse;

    emit sendState(CAPTURING);
    LOG_INFO("Starting Calibration");
}

void cCameraCalibrationFilter::OnStartCalibrationFisheye()
{
    resetCalibrationResults();
    // switch state to CAPTURING
    m_calibrationState = CAPTURING;
    m_calibrationSettings.flag = 0;
    m_calibrationSettings.flag = fisheye::CALIB_RECOMPUTE_EXTRINSIC /*| fisheye::CALIB_CHECK_COND */ | fisheye::CALIB_FIX_SKEW;
    m_calibrationSettings.useFisheye = tTrue;
    emit sendState(CAPTURING);
    LOG_INFO("Starting Calibration Fisheye Camera");
}

void cCameraCalibrationFilter::OnSaveAs(QString qFilename)
{
    // save the calibration results to the given file in argument
    if (m_calibrationState == CAPTURING_FINISHED)
    {

        // convert file name to absolute file
        cFilename filename = qFilename.toStdString().c_str();
        //ADTF_GET_CONFIG_FILENAME(filename);
        filename = filename.CreateAbsolutePath(".");
        m_calibrationSettings.outputFileName = filename;
        LOG_INFO(cString::Format("Attempt to save calibration to %s", filename.GetPtr()));

        if (runCalibrationAndSave(m_matrixSize, m_cameraMatrix, m_distCoeffs, m_imagePoints))
        {
            LOG_INFO(cString::Format("Saved calibration to %s", filename.GetPtr()));
            // switch to state WAITING again
            emit sendMessage("");
            m_calibrationState = WAITING;
        }
    }
}

void cCameraCalibrationFilter::OnCancel()
{
    m_calibrationState = WAITING;
    emit sendState(m_calibrationState);
    emit sendMessage("");
}