

#include <stdio.h>
#include <stdlib.h>

//#include "windows.h"
#include <stdio.h>
//#include <tchar.h>

#include "sensorData.h"
#include "opencv2/opencv.hpp"


int mainCreator(int argc, char *argv[]) {
    ml::SensorData sd;
    const std::string prefix = "/home/leoneed/CLionProjects/BundleFusion_Ubuntu_V0/data/sequenceCR.sens/frame-00000";

    for (int frame = 0; frame <= 6; ++frame){
        uint w = 640;
        uint h = 480;
        const std::string imagePairNumber = prefix + std::to_string(frame);
        std::string dPath = imagePairNumber + ".depth.png";
        std::string rgbPath = imagePairNumber + ".color.jpg";

        cv::Mat rgb = cv::imread(rgbPath);
        cv::Mat d = cv::imread(dPath, cv::IMREAD_GRAYSCALE);

        std::vector<unsigned short> depthImageRaw;
        {
            for (ushort i = 0; i < d.cols; ++i) {
                for (ushort j = 0; j < d.rows; ++j) {
                    assert(i < w);
                    assert(j < h);

                    unsigned int pixelD = d.at<ushort>(j, i);
                    assert(pixelD < 66000);
                    assert(pixelD >= 0);
                    depthImageRaw.push_back(static_cast<ushort> (pixelD));
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
//    std::vector<ml::vec3uc> rgbVec;
//    rgbVec.reserve(rgb.rows * rgb.cols);

        std::cout << "try adding frame" << std::endl;
        ml::SensorData::CalibrationData cdRgb;
        cdRgb.setMatrices(ml::SensorData::CalibrationData::makeIntrinsicMatrix(583.0, 583.0, 320.0, 340.0));
        ml::SensorData::CalibrationData cdD;
        cdD.setMatrices(ml::SensorData::CalibrationData::makeIntrinsicMatrix(583.0, 583.0, 320.0, 340.0));

        sd.initDefault(w, h, w, h,
                       cdRgb,
                       cdD,
                       ml::SensorData::TYPE_RAW,
                       ml::SensorData::TYPE_RAW_USHORT);
        sd.addFrame((ml::vec3uc *) rgbImageRaw.data(), depthImageRaw.data());
        //    sd.addFrame((ml::vec3uc *) rgb.data, (unsigned short *) d.data);
        std::cout << "frame added " << frame << std::endl;
    }

    sd.saveToFile("/home/leoneed/CLionProjects/BundleFusion_Ubuntu_V0/data/exported6.sens");
    return 0;
}

int mainReader(int argc, char *argv[]) {
//#if defined(DEBUG) | defined(_DEBUG)
//	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
//#endif
    //_CrtSetBreakAlloc(7545);
    try {
        //non-cached read
        const std::string filename = "/home/leoneed/CLionProjects/BundleFusion_Ubuntu_V0/data/exported6.sens";
        ml::SensorData sd;

        std::cout << "loading from file... ";
        sd.loadFromFile(filename);
        std::cout << "done!" << std::endl;

        std::cout << sd << std::endl;

        for (size_t i = 0; i < sd.m_frames.size(); i++) {
            std::cout << "\r[ processing frame " << std::to_string(i) << " of " << std::to_string(sd.m_frames.size())
                      << " ]";
            ml::vec3uc *colorData = sd.decompressColorAlloc(i);
            unsigned short *depthData = sd.decompressDepthAlloc(i);

            sd.m_colorWidth;
            sd.m_colorHeight;
            sd.m_depthWidth;
            sd.m_depthHeight;

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

    if (save) {
        return mainCreator(argc, argv);
    } else {
        return mainReader(argc, argv);
    }
}
