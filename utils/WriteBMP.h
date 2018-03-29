#ifndef WRITEBMP_H
#define WRITEBMP_H

#include <fstream>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <cmath>
#include <algorithm>

/* using namespace std; */

class WriteBMP
{
    private:
        unsigned char file[14] = {
            'B','M',         // bftype - specifies its bmp file
            0,0,0,0,         // bfsize - size of the whole bmp in bytes
            0,0,             // bfreserved1 - app data (must be zero)
            0,0,             // bfreserved2 - app data (must be zero)
            40+14,0,0,0      // bfoffbits - distance to the begining of the image data in bytes (data offset)
        };

        unsigned char info[40] = {
            40,0,0,0,        // bisize - info hd size
            0,0,0,0,         // biwidth - image width in pixels
            0,0,0,0,         // biheight - image heigth in pixels
            1,0,             // biplanes - number color planes
            24,0,            // bibitcount - bits per pixel
            0,0,0,0,         // bicompression - compression is none
            0,0,0,0,         // bisizeimage - image size in bytes
            0x13,0x0B,0,0,   // bixpelspermeter - horz resoluition in pixel / m
            0x13,0x0B,0,0,   // biypelspermeter - vert resolutions (0x03C3 = 96 dpi, 0x0B13 = 72 dpi)
            0,0,0,0,         // #colors in pallete
            0,0,0,0,         // #important colors
        };

        unsigned char pad[3] = {0,0,0};
        unsigned char pixel[3];

        int padSize;
        int imageSize;
        int totalSize;
        int row;
        std::ofstream myfile;

        long red, green, blue;
        std::vector<int> v1;
        std::vector<int> v2;
        std::vector<int> v3;

        Eigen::RowVector2d pt1, pt2, vec, newpt;
        Eigen::RowVector3d temp;

    public:
        WriteBMP(int width, int height, std::vector<Eigen::RowVector3d> vertices, std::vector<Eigen::RowVector3d> color, std::vector<Eigen::RowVector2d> edges, std::string filename);

};

#endif 
