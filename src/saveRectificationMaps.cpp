#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

#include <ladybug2/Ladybug2.h>
#include <ladybug2/rectificationMaps.h>

void matwrite(const std::string& filename, const cv::Mat& mat)
{
    std::ofstream fs(filename.c_str(), std::fstream::binary);

    // Header
    int type = mat.type();
    int channels = mat.channels();
    fs.write((char*)&mat.rows, sizeof(int));    // rows
    fs.write((char*)&mat.cols, sizeof(int));    // cols
    fs.write((char*)&type, sizeof(int));        // type
    fs.write((char*)&channels, sizeof(int));    // channels

    // Data
    if (mat.isContinuous())
    {
        fs.write(mat.ptr<char>(0), (mat.dataend - mat.datastart));
    }
    else
    {
        int rowsz = CV_ELEM_SIZE(type) * mat.cols;
        for (int r = 0; r < mat.rows; ++r)
        {
            fs.write(mat.ptr<char>(r), rowsz);
        }
    }
}

cv::Mat matread(const std::string& filename)
{
    std::ifstream fs(filename.c_str(), std::fstream::binary);

    // Header
    int rows, cols, type, channels;
    fs.read((char*)&rows, sizeof(int));         // rows
    fs.read((char*)&cols, sizeof(int));         // cols
    fs.read((char*)&type, sizeof(int));         // type
    fs.read((char*)&channels, sizeof(int));     // channels

    // Data
    cv::Mat mat(rows, cols, type);
    fs.read((char*)mat.data, CV_ELEM_SIZE(type) * rows * cols);

    return mat;
}

cv::Mat vector2mat(int rows, int cols, int *vector)
{
    cv::Mat mat(rows, cols, CV_16SC1);
    int k = 0;
    for(int i=0; i<rows; i++)
    {
        for(int j=0; j<cols; j++)
        {
            mat.at<int>(i, j) = vector[k];            
            k++;
        }
    }
    return mat;
}

cv::Mat int2float(cv::Mat matInt)
{
    cv::Mat matFloat(matInt.rows, matInt.cols, CV_32FC1);

    for(int i=0; i<matInt.rows; i++)
    {
        for(int j=0; j<matInt.cols; j++)
        {
            matFloat.at<float>(i, j) = (float) (matInt.at<int>(i, j));
        }
    }
    return matFloat;
}

cv::Mat transposeAndMirrow(cv::Mat mat)
{
    cv::Mat matTrans(mat.cols, mat.rows, CV_32FC1);

    for(int i=0; i<mat.rows; i++)
    {
        for(int j=0; j<mat.cols; j++)
        {
            matTrans.at<float>(j, i) = mat.at<float>(i, j);
        }
    }

    cv::Mat matMirrowed(matTrans.rows, matTrans.cols, CV_32FC1);
    for(int i=0; i<matTrans.rows; i++)
    {
        for(int j=0; j<matTrans.cols; j++)
        {
            matMirrowed.at<float>(i, j) = matTrans.at<float>(i, matTrans.cols - j - 1);
        }
    }

    return matMirrowed;
}

int main(int argc, char **argv)
{
    cv::Mat map0c, map0c_f, map0c_f_tr;
    map0c = vector2mat(1024, 768, vec_rectColsCam0);
    map0c_f = int2float(map0c);
    //map0c_f_tr = transposeAndMirrow(map0c_f);
    matwrite("src/ladybug2/binary_files/bin_rectColsCam0.bin", map0c_f);

    cv::Mat map0r, map0r_f, map0r_f_tr;
    map0r = vector2mat(1024, 768, vec_rectRowsCam0);
    map0r_f = int2float(map0r);
    //map0c_f_tr = transposeAndMirrow(map0c_f);
    matwrite("src/ladybug2/binary_files/bin_rectRowsCam0.bin", map0r_f);


    cv::Mat map1c, map1c_f, map1c_f_tr;
    map1c = vector2mat(768, 1024, vec_rectColsCam1);
    map1c_f = int2float(map1c);
    map1c_f_tr = transposeAndMirrow(map1c_f);
    matwrite("src/ladybug2/binary_files/bin_rectColsCam1.bin", map1c_f_tr);

    cv::Mat map1r, map1r_f, map1r_f_tr;
    map1r = vector2mat(768, 1024, vec_rectRowsCam1);
    map1r_f = int2float(map1r);
    map1r_f_tr = transposeAndMirrow(map1r_f);
    matwrite("src/ladybug2/binary_files/bin_rectRowsCam1.bin", map1r_f_tr);


    cv::Mat map2c, map2c_f, map2c_f_tr;
    map2c = vector2mat(768, 1024, vec_rectColsCam2);
    map2c_f = int2float(map2c);
    map2c_f_tr = transposeAndMirrow(map2c_f);
    matwrite("src/ladybug2/binary_files/bin_rectColsCam2.bin", map2c_f_tr);

    cv::Mat map2r, map2r_f, map2r_f_tr;
    map2r = vector2mat(768, 1024, vec_rectRowsCam2);
    map2r_f = int2float(map2r);
    map2r_f_tr = transposeAndMirrow(map2r_f);
    matwrite("src/ladybug2/binary_files/bin_rectRowsCam2.bin", map2r_f_tr);


    cv::Mat map3c, map3c_f, map3c_f_tr;
    map3c = vector2mat(768, 1024, vec_rectColsCam3);
    map3c_f = int2float(map3c);
    map3c_f_tr = transposeAndMirrow(map3c_f);
    matwrite("src/ladybug2/binary_files/bin_rectColsCam3.bin", map3c_f_tr);

    cv::Mat map3r, map3r_f, map3r_f_tr;
    map3r = vector2mat(768, 1024, vec_rectRowsCam3);
    map3r_f = int2float(map3r);
    map3r_f_tr = transposeAndMirrow(map3r_f);
    matwrite("src/ladybug2/binary_files/bin_rectRowsCam3.bin", map3r_f_tr);


    cv::Mat map4c, map4c_f, map4c_f_tr;
    map4c = vector2mat(768, 1024, vec_rectColsCam4);
    map4c_f = int2float(map4c);
    map4c_f_tr = transposeAndMirrow(map4c_f);
    matwrite("src/ladybug2/binary_files/bin_rectColsCam4.bin", map4c_f_tr);

    cv::Mat map4r, map4r_f, map4r_f_tr;
    map4r = vector2mat(768, 1024, vec_rectRowsCam4);
    map4r_f = int2float(map4r);
    map4r_f_tr = transposeAndMirrow(map4r_f);
    matwrite("src/ladybug2/binary_files/bin_rectRowsCam4.bin", map4r_f_tr);


    cv::Mat map5c, map5c_f, map5c_f_tr;
    map5c = vector2mat(768, 1024, vec_rectColsCam5);
    map5c_f = int2float(map5c);
    map5c_f_tr = transposeAndMirrow(map5c_f);
    matwrite("src/ladybug2/binary_files/bin_rectColsCam5.bin", map5c_f_tr);

    cv::Mat map5r, map5r_f, map5r_f_tr;
    map5r = vector2mat(768, 1024, vec_rectRowsCam5);
    map5r_f = int2float(map5r);
    map5r_f_tr = transposeAndMirrow(map5r_f);
    matwrite("src/ladybug2/binary_files/bin_rectRowsCam5.bin", map5r_f_tr);

}