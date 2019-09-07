#include "pch.h"

cv::Mat gammaCorrection(const cv::Mat &img, const double gamma_) {
  CV_Assert(gamma_ >= 0);
  cv::Mat lookUpTable(1, 256, CV_8U);
  uchar *p = lookUpTable.ptr();
  for (int i = 0; i < 256; ++i) {
    p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 1 / gamma_) * 255.0);
  }
  cv::Mat res = img.clone();
  LUT(img, lookUpTable, res);
  return res;
}

std::string concatPaths(std::string folder, std::string file_name) {
  std::string output = folder + "/" + file_name;
  return output;
}

bool getFileNames(std::string dir, std::vector<std::string> &fileNames,
                  std::string file_type) {
  for (const std::experimental::filesystem::directory_entry &i :
       std::experimental::filesystem::directory_iterator(dir)) {
    std::string file = i.path().filename().string();
    int pos = file.rfind(file_type);
    if (pos == std::string::npos) {
      std::cout << "File not found." << std::endl;
    } else {
      fileNames.push_back(i.path().filename().string());
    }
  }
  if (fileNames.size() == 0) return false;
  return true;
}

std::vector<cv::Mat> readImages(std::string dir, std::string fileType) {
  std::vector<cv::Mat> srcImages;
  std::vector<std::string> fileNames;
  getFileNames(dir, fileNames, fileType);
  for (std::string f : fileNames) {
    std::string file_path = concatPaths(dir, f);
    cv::Mat image = cv::imread(file_path);
    if (image.empty()) {
      std::cerr << "Cannot load image: " << file_path << std::endl;
    } else {
      srcImages.push_back(image);
    }
  }
  return srcImages;
}

std::vector<cv::Mat> readImages(std::string dir, std::string fileType,
                                std::vector<std::string> &fileNames) {
  std::vector<cv::Mat> srcImages;
  getFileNames(dir, fileNames, fileType);
  for (std::string f : fileNames) {
    std::string file_path = concatPaths(dir, f);
    cv::Mat image = cv::imread(file_path);
    if (image.empty()) {
      std::cerr << "Cannot load image: " << file_path << std::endl;
    } else {
      srcImages.push_back(image);
    }
  }
  return srcImages;
}

int calibImages(std::vector<cv::Mat> images, std::vector<cv::Mat> &drawedimages,
                int rowNum, int colNum, float chessSize,
                std::string paramFileName) {
  cv::Size pattern_size = cv::Size2i(colNum, rowNum);
  std::vector<cv::Point2f> corners;
  std::vector<std::vector<cv::Point2f>> image_points;
  std::vector<cv::Point3f> object;
  std::vector<std::vector<cv::Point3f>> object_points;
  cv::Mat cameraMatrix, distortionCoefficients;
  std::vector<cv::Mat> rvecs, tvecs;
  for (int i = 0; i < rowNum; i++) {
    for (int j = 0; j < colNum; j++) {
      cv::Point3f p(i * chessSize, j * chessSize, 0.0);
      object.push_back(p);
    }
  }
  int foundImagesNum = 0;
  cv::namedWindow("check window", cv::WINDOW_AUTOSIZE);
  for (cv::Mat image : images) {
    bool found = cv::findChessboardCorners(image, pattern_size, corners);
    cv::Mat dst = image.clone();
    cv::drawChessboardCorners(dst, pattern_size, corners, found);
    drawedimages.push_back(dst);
    if (found) {
      foundImagesNum++;
      image_points.push_back(corners);
      object_points.push_back(object);
    }
    cv::imshow("check window", dst);
    cv::waitKey(500);
  }
  cv::destroyAllWindows();
  if (foundImagesNum == 0) return -1;
  double retval =
      cv::calibrateCamera(object_points, image_points, drawedimages[0].size(),
                          cameraMatrix, distortionCoefficients, rvecs, tvecs);
  cv::FileStorage fs(paramFileName, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cerr << "File can not be opened." << std::endl;
    return -1;
  }
  fs << "rms" << retval;
  fs << "intrinsic" << cameraMatrix;
  fs << "distortion" << distortionCoefficients;
  fs.release();
  return 0;
}

int main(int argc, char **argv) {
  std::string folder_name = "images";
  std::string file_type = "jpg";
  int chessRow = 8;
  int chessColumn = 6;
  float chessSize = 23.0;
  std::string outputParamFileName = "camera.xml";
  std::vector<cv::Mat> images = readImages(folder_name, file_type);
  std::vector<cv::Mat> dsts;
  calibImages(images, dsts, chessRow, chessColumn, chessSize,
              outputParamFileName);
  return 0;
}
