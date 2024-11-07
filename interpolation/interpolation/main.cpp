#include <opencv2/opencv.hpp>
#include <iostream>
#include <optional>

/*
    Derivation
    ----------

         x-tl    tr-x
         fract   1-fract
        tl---|------------tr  }
        |    |            |   }
        |    |            |   } fract
        |----|------------|   }
        bl---|------------br    1-fract

        val_x_top = (f(tl) * (tr - x) + f(tr) * (x - tl)) / tr-tl (which is 1 for the pixel domain)
        val_x_bottom = (f(bl) * (tr - x) + f(br) * (x-tl)) / tr-tl

        Let FACTOR = 1 / ((tr-tl) * (bl-tl)) = 1 as we are working in pixel domain

        val_x_y = ( f(tl) * (1 - x_fract) * (1 - y_fract) +  --> top got 1 - y fract
                    f(tr) * (x_fract) * (1 - y_fract) +   --> top got 1 - y fract
                    f(bl) * (1 - x_fract) * (y_fract) +    ---> bottom got y fract
                    f(br) * (x_fract) * (y_fract) ) * FACTOR --> bottom got y fract
*/
std::optional<double> bilinearInterpolation(const cv::Mat& image, double x, double y) {
    // Ensure coordinates are within image bound
    if (x < 0 || x >= image.cols-1 || y < 0 || y >= image.rows-1) {
        std::cerr << "Coordinates are out of range!" << std::endl;
        return {};
    }

    // Get integer parts of x and y
    int x_ = static_cast<int>(x);
    int y_ = static_cast<int>(y);

    // Get fraction part of x and y
    double x_fract = x - x_;
    double y_fract = y - y_;

    // Pixel values for the 2x2 neighbourhood
    double tl = static_cast<double>(image.at<uchar>(y_, x_));
    double tr = static_cast<double>(image.at<uchar>(y_, x_ + 1));
    double bl = static_cast<double>(image.at<uchar>(y_ + 1, x_));
    double br = static_cast<double>(image.at<uchar>(y_ + 1, x_ + 1));
   
    // Bilinear interpolation
    double interpolatedValue =
            tl * (1 - x_fract) * (1 - y_fract) + 
            tr * x_fract * (1-y_fract) +
            bl * (1 - x_fract) * y_fract +
            br * x_fract * y_fract;

    return {interpolatedValue};
}

int main(int argc, char* argv[]) {
    cv::Mat image = cv::imread("moodeng.jpg", cv::IMREAD_GRAYSCALE);
    if (!image.data) {
        std::cerr << "Cannot load input image" << std::endl;
        return -1;
    }

    // Define subpixel coordinate (example)
    double x = 50.75;
    double y = 100.75;

    std::cout << "tl: " << static_cast<int>(image.at<uchar>((int)y, (int)x)) << std::endl;
    std::cout << "tr: " << static_cast<int>(image.at<uchar>((int)y, (int)x+1)) << std::endl;
    std::cout << "bl: " << static_cast<int>(image.at<uchar>((int)y+1, (int)x)) << std::endl;
    std::cout << "br: " << static_cast<int>(image.at<uchar>((int)y+1, (int)x+1)) << std::endl;

    auto interpolatedVal = bilinearInterpolation(image, x, y);
    if (interpolatedVal) {
        std::cout << "Interpolated value: " << *interpolatedVal << std::endl;
    }

    return 0;
}
