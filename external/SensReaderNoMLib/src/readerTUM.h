//
// Created by leoneed on 12.04.2021.
//

#ifndef READERSENS_READERTUM_H
#define READERSENS_READERTUM_H

#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include "boost/filesystem.hpp"
#include "opencv2/opencv.hpp"

struct pairedImages {
    double timeRGB;
    double timeD;
    std::string pathRGB;
    std::string pathD;

    pairedImages(double tRGB, double tD, const std::string &rgb, const std::string &d)
            : timeRGB(tRGB),
              timeD(tD),
              pathRGB(rgb),
              pathD(d) {}
};

struct readerTUM {
    std::string datasetPath;
    std::string bundleFusionPath;
    std::string assocFile = "assoc.txt";
    std::string rgb = "rgb";
    std::string depth = "depth";

    readerTUM(const std::string &datasetPathToSet,
              const std::string &bundleFusionPathToSet) :
            datasetPath(datasetPathToSet),
            bundleFusionPath(bundleFusionPathToSet) {};

    std::vector<pairedImages> pairImages(int numberOfImages, bool saveImages) {

        std::vector<pairedImages> paired;
        const std::string sep = "/";
        const std::string assocPath = datasetPath + sep + assocFile;
        std::string currentLine;
        std::ifstream assocFileTimeRGBTimeD(assocPath);

        int counter  = 0;
        while (std::getline(assocFileTimeRGBTimeD, currentLine)) {
            if (currentLine[0] == '#') {
                continue;
            }

            if (counter >= numberOfImages) {
                break;
            }

            std::stringstream pairInformation;

            double timestampRGB = 0.0;
            double timeStampD = 0.0;
            std::string pathRGB = "";
            std::string pathD = "";

            pairInformation << currentLine;

            pairInformation >> timestampRGB;
            pairInformation >> pathRGB;
            pairInformation >> timeStampD;
            pairInformation >> pathD;

            std::string pathRGBFull = datasetPath + sep + pathRGB;
            std::string pathDFull = datasetPath + sep + pathD;

            std::string numberString = std::to_string(counter);
            while (numberString.length() < 6) {
                numberString = "0" + numberString;
            }


            std::string rgbShort = numberString + "." + "color" + "." + "png";
            std::string dShort = numberString + "." + "depth" + "." + "png";
            std::string rgbToSave = bundleFusionPath + sep + rgbShort;
            std::string dToSave = bundleFusionPath + sep + dShort;

            if (saveImages) {
                boost::filesystem::copy(pathRGBFull, rgbToSave);
                std::cout << "read depth " << pathDFull << std::endl;
                cv::Mat depthImage = cv::imread(pathDFull, cv::IMREAD_GRAYSCALE);
                cv::Mat depthImageReversed = depthImage;
                for (int i = 0; i < 640; ++i) {
                    for (int j = 0; j < 480; ++j) {
                        ushort depthV = depthImage.at<ushort>(j, i);
                        assert(std::numeric_limits<ushort>::max() == 65535);
                        assert(depthV >= 0 && depthV <= std::numeric_limits<ushort>::max());
                        depthImageReversed.at<ushort>(j, i) = static_cast<ushort>(depthV);
                    }
                }
                std::cout << "now save by " << dToSave << std::endl;
                cv::imwrite(dToSave, depthImageReversed);
                std::cout << "saved" << std::endl;
//                boost::filesystem::copy(pathDFull, dToSave);
            }
            paired.emplace_back(pairedImages(timestampRGB, timeStampD, rgbShort, dShort));
            ++counter;
        }

        return paired;
    }


};

#endif //READERSENS_READERTUM_H
