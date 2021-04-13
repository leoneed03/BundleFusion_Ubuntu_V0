
#include <Eigen/Eigen>
#include <stdlib.h>
#include <cmath>
#include <stdio.h>
#include <fstream>
#include "sensorData.h"
#include "opencv2/opencv.hpp"
#include "readerTUM.h"

// first argument is cameraIntr.txt -- file with camera intrinsics

int mainCreator(int argc, char *argv[], const std::vector<pairedImages> &pairedImages,
                const std::string& pathBFdata) {
    std::string cameraIntrPath = "../cameraIntrDefault.txt";
    if (argc > 1) {
        cameraIntrPath = std::string(argv[1]);
    } else {
    }

    double fx = 517.3, fy = 516.5, cx = 318.6, cy = 255.3;
    std::cout << "camera intrinsics file with fx, fy, cx, cy is " << cameraIntrPath << std::endl;
    std::fstream cameraIntrStream(cameraIntrPath);
    if (cameraIntrStream.is_open()) {
        cameraIntrStream >> fx;
        cameraIntrStream >> fy;
        cameraIntrStream >> cx;
        cameraIntrStream >> cy;
    } else {
        std::cout << "CAMERA FILE NOT FOUND by " << cameraIntrPath << std::endl;
        std::cout << "using default instrinsics fx, fy, cx, cy: " << fx << ' ' << fy << ' ' << cx << ' ' << cy << ' ' << std::endl;
    }
    ml::SensorData sd;
    const std::string prefix = pathBFdata + "/";

    int iter = pairedImages.size();
    assert(iter < 20);


    for (int frame = 0; frame < iter; ++frame){
        std::string number = std::to_string(frame);
        while (number.length() < 6) {
            number = "0" + number;
        }
        uint w = 640;
        uint h = 480;
        const std::string imagePairNumber = prefix + number;
        std::string dPath = imagePairNumber + ".depth.png";
        std::string rgbPath = imagePairNumber + ".color.png";
        std::string imageRGBjpg = rgbPath;//imagePairNumber + ".color.png";
//        {
//
//            cv::Mat rgbPNG = cv::imread(rgbPath);
//            std::cout << "read png by " << rgbPath << std::endl;
//            assert(!rgbPNG.empty());
//            cv::imwrite(imageRGBjpg, rgbPNG);
//            boost::filesystem::remove(rgbPath);
//        }

//        std::string imageRGBjpg = rgb;
        cv::Mat rgb = cv::imread(imageRGBjpg);
        cv::Mat d = cv::imread(dPath, cv::IMREAD_GRAYSCALE);
        std::cout << "rgb for path " << imageRGBjpg << std::endl;
        std::cout << "d fro path " << dPath << std::endl;
        assert(!rgb.empty());
        assert(!d.empty());

        std::vector<unsigned short> depthImageRaw;
        {
            for (ushort i = 0; i < d.cols; ++i) {
                for (ushort j = 0; j < d.rows; ++j) {
                    assert(i < w);
                    assert(j < h);

                    unsigned int pixelD = d.at<ushort>(j, i);
                    assert(pixelD < 66000);
                    assert(pixelD >= 0);
                    ushort pixelMultiplied = pixelD;
                    assert(pixelMultiplied >= 0 && pixelMultiplied <= 65535);
                    depthImageRaw.push_back(pixelMultiplied);
                }
            }
        }

        std::vector<uchar> rgbImageRaw;
        std::vector<cv::Vec<u_char, 3>> rgbCV;

        {
            for (ushort i = 0; i < rgb.cols; ++i) {
                for (ushort j = 0; j < rgb.rows; ++j) {
                    assert(i < w);
                    assert(j < h);

                    auto pixelRGB = rgb.at<cv::Vec3b>(j, i);
                    rgbCV.emplace_back(pixelRGB);
                }
            }
            for (auto &e: rgbCV) {
                for (int i = 0; i < 3; ++i) {
                    rgbImageRaw.push_back(e[i]);
                }
            }
        }
        assert(rgbCV.size() == w * h);
        assert(rgbImageRaw.size() == 3 * rgbCV.size());
        assert(rgbImageRaw.size() == 3 * depthImageRaw.size());
        assert(!rgbImageRaw.empty());
        std::cout << "try adding frame" << std::endl;
        std::cout << "with intr " << fx << ' ' << fy << ' ' << cx << ' ' << cy << std::endl;
        ml::SensorData::CalibrationData cdRgb;
        cdRgb.setMatrices(ml::SensorData::CalibrationData::makeIntrinsicMatrix(fx, fy, cx, cy));
        ml::SensorData::CalibrationData cdD;
        cdD.setMatrices(ml::SensorData::CalibrationData::makeIntrinsicMatrix(fx, fy, cx, cy));

        sd.initDefault(w, h, w, h,
                       cdRgb,
                       cdD,
                       ml::SensorData::TYPE_RAW,
                       ml::SensorData::TYPE_RAW_USHORT
                       ,(5000.0)
                       );

        sd.addFrame((ml::vec3uc *) rgb.data, (unsigned short *) d.data);
//        sd.addFrame((ml::vec3uc *) rgbImageRaw.data(), depthImageRaw.data());
        sd.m_frames[frame].setTimeStampColor(pairedImages[frame].timeRGB * 1000);
        sd.m_frames[frame].setTimeStampDepth(pairedImages[frame].timeD * 1000);

        double timediff = std::abs(pairedImages[frame].timeRGB - pairedImages[frame].timeD);
        assert(timediff < 0.02);
        //    sd.addFrame((ml::vec3uc *) rgb.data, (unsigned short *) d.data);
        std::cout << "frame added " << frame << std::endl;
    }

    sd.saveToFile("/home/leoneed/CLionProjects/BundleFusion_Ubuntu_V0/data/sequence.sens");
    return 0;
}

// first arg is .sens sequence
// second arg is poses.txt file path (SE3 coordinates)
int mainReader(int argc, char *argv[]) {
    std::cout << "start reading" << std::endl;
    try {
        //non-cached read
        std::string filename = "/home/leoneed/CLionProjects/BundleFusion_Ubuntu_V0/data/exported6.sens";
        std::string posesFile = "../poses.txt";

        if (argc > 1) {
            filename = std::string(argv[1]);
        }

        if (argc > 2) {
            posesFile = std::string(argv[2]);
        }

        std::cout << "now .sens file is " << filename << std::endl;
        std::cout << "saving trajectory to " << posesFile << std::endl;
        ml::SensorData sd;

        std::cout << "loading from file... ";
        sd.loadFromFile(filename);
        std::cout << "done!" << std::endl;

        std::cout << sd << std::endl;
        std::ofstream posesOut(posesFile);
        if (!posesOut.is_open()) {
            std::cout << "POSES FILE NOT OPEN" << std::endl;
        }
        assert(posesOut.is_open());

        for (size_t i = 0; i < sd.m_frames.size(); i++) {
            std::cout << "\r[ processing frame " << std::to_string(i) << " of " << std::to_string(sd.m_frames.size())
                      << " ]";
            auto& frameCurr = sd.m_calibrationColor;

            ml::vec3uc *colorData = sd.decompressColorAlloc(i);
            unsigned short *depthData = sd.decompressDepthAlloc(i);

            sd.m_colorWidth;
            sd.m_colorHeight;
            sd.m_depthWidth;
            sd.m_depthHeight;

            std::cout << "depth shift (depth_in_meters = sd.m_depthShift * depthData[0]) " << sd.m_depthShift << std::endl;

            auto& cameraToWorld = sd.m_frames[i].getCameraToWorld();
            Eigen::Matrix3f rotation;
            rotation.setIdentity();
            Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> matrixView((float *) &cameraToWorld);
            Eigen::Matrix3f rotationView(matrixView.topLeftCorner<3,3>());
            cameraToWorld.printToConsole();
            std::cout << "vs read rot" << std::endl;
            std::cout << rotationView << std::endl;
            Eigen::Quaternionf rotationQuat(rotationView);
            rotationQuat.normalize();
            Eigen::Vector3f translationVec = matrixView.topRightCorner<3, 1>();
            std::cout << "translation vector is " << std::endl;
            std::cout << translationVec << std::endl;
            assert(cameraToWorld._m01 == rotationView.col(1)[0]);
            assert(cameraToWorld._m20 == rotationView.col(0)[2]);
            assert(cameraToWorld._m23 == translationVec[2]);

            double timeColour = sd.m_frames[i].getTimeStampColor() / 1000.0;
            double timeDepth = sd.m_frames[i].getTimeStampDepth() / 1000.0;
            std::cout << timeColour << " vs time depth " << timeDepth << std::endl;
            double timediff = timeColour - timeDepth;
            if (timediff < 0) {
                timediff = -timediff;
            }
            std::cout << "timediff is " << timediff << std::endl;
            assert(timediff >= 0);
            assert(timediff < 0.02);

            posesOut << timeDepth << ' ';
            posesOut << translationVec[0] << ' ' << translationVec[1] << ' ' << translationVec[2] << ' ';
            posesOut << rotationQuat.x() << ' ' << rotationQuat.y() << ' ' << rotationQuat.z() << ' ' << rotationQuat.w() << std::endl;

            //convert to m:
            float depth_in_meters = sd.m_depthShift * depthData[0];


            std::free(colorData);
            std::free(depthData);
        }
        std::cout << std::endl;
    }
    catch (const std::exception &e) {
//		MessageBoxA(NULL, e.what(), "Exception caught", MB_ICONERROR);
        exit(EXIT_FAILURE);
    }
    catch (...) {
//		MessageBoxA(NULL, "UNKNOWN EXCEPTION", "Exception caught", MB_ICONERROR);
        exit(EXIT_FAILURE);
    }

    std::cout << "<press key to continue>" << std::endl;
    getchar();
    return 0;
}


int main(int argc, char *argv[]) {
    bool save = true;
    std::string lastArg = std::string(argv[argc - 1]);
    assert(argc > 1);


    if (lastArg == "CREATE") {

        std::string dataset = "/home/leoneed/Desktop/datasets/freiburg/rgbd_dataset_freiburg1_plant";//"/home/leoneed/Desktop/datasets/freiburg/fr1_desk_short";
        std::string bf = "/home/leoneed/CLionProjects/BundleFusion_Ubuntu_V0/data/TUM1";
        int numberOfImages = 4;
        readerTUM readerTum(dataset, bf);
        std::vector<pairedImages> pairings = readerTum.pairImages(numberOfImages, true);
//
//
//        for (const auto& e: pairings) {
//            std::cout << e.timeRGB << "->" << e.pathRGB << " & " << e.timeD << "->" << e.pathD << std::endl;
//        }
//        std::cout << "SIZE " << pairings.size() << std::endl;
//        std::cout << std::endl << std::endl;


        return mainCreator(argc, argv, pairings, bf);
    } else if (lastArg == "READ") {
        return mainReader(argc, argv);
    } else {
        assert(false && "use last arg: CREATE for .sens file creation and READ for reading .sens ");
        return 1;
    }
}
