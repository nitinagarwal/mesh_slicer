#include "WriteBMP.h"

WriteBMP::WriteBMP(int width, int height, std::vector<Eigen::RowVector3d> vertices, std::vector<Eigen::RowVector3d> color, std::vector<Eigen::RowVector2d> edges, std::string filename){

    /* assuming the input vertices are already aligned and have constant z value */

    padSize  = (4- (( width*3 ) %4 ));  // have to make sure the width is multiple of 4
    if(padSize==4){
        padSize=0;
    }

    imageSize = (width * 3 + padSize) * height;          // size of the image in bytes
    totalSize  = imageSize + sizeof(file) + sizeof(info);  // total size in bytes

    file[ 2] = (unsigned char)( totalSize    );
    file[ 3] = (unsigned char)( totalSize>> 8);
    file[ 4] = (unsigned char)( totalSize>>16);
    file[ 5] = (unsigned char)( totalSize>>24);

    info[ 4] = (unsigned char)( width   );
    info[ 5] = (unsigned char)( width>> 8);
    info[ 6] = (unsigned char)( width>>16);
    info[ 7] = (unsigned char)( width>>24);

    info[ 8] = (unsigned char)( height    );
    info[ 9] = (unsigned char)( height>> 8);
    info[10] = (unsigned char)( height>>16);
    info[11] = (unsigned char)( height>>24);

    info[20] = (unsigned char)( imageSize    );
    info[21] = (unsigned char)( imageSize>> 8);
    info[22] = (unsigned char)( imageSize>>16);
    info[23] = (unsigned char)( imageSize>>24);

    double z_value = vertices.at(0)(2); // constant z value

    /* Interpolating the edges so that contours are closed */
    for(int i=0;i<edges.size();i++){

        pt1 << vertices.at(round(edges.at(i)(0)))(0),
                    vertices.at(round(edges.at(i)(0)))(1);
        
        pt2 << vertices.at(round(edges.at(i)(1)))(0),
                    vertices.at(round(edges.at(i)(1)))(1);
        
        vec = pt2 - pt1; // vector for parameterization */

        for(double t=0.1; t<1.0; t=t+0.1){

            newpt = pt1 + t*vec;
            temp << newpt(0), newpt(1), z_value;
            vertices.push_back(temp);

            temp << color.at(round(edges.at(i)(0)))(0), color.at(round(edges.at(i)(0)))(1), 
               color.at(round(edges.at(i)(0)))(2);
            color.push_back(temp);
        }
    }
    /* std::cout << "vertices size " << vertices.size() << " color size " << color.size() << std::endl; */

    for(int i=0;i<vertices.size();i++){
        v1.push_back(round(vertices.at(i)(0)));
        v2.push_back(round(vertices.at(i)(1)));
        v3.push_back(round(vertices.at(i)(2)));
    }

    /* writing the binary data into the bmp file */
    myfile.open(filename);

    myfile.write( (char*)file, sizeof(file) );
    myfile.write( (char*)info, sizeof(info) );

    std::vector<int>::iterator it1;

    for ( int y=0; y<height; y++) //height-1; y>=0; y-- )
    {
        for ( int x=0; x<width; x++ )
        {

            it1 = find(v1.begin(),v1.end(),x);

            red = 255; // else if not found
            green = 255;
            blue = 255;

            /* cout << "x= " << x << "   y=  " << y << endl; */
            while(it1 != v1.end()){

                row = distance(v1.begin(),it1);
                /* cout << "Value at index " << row << "is  " << *it1 << endl; */
                if( v2[row] == y){

                    //std::cout << "found " << std::endl;
                    red = round(color.at(row)(0) * 255); // else if not found */ */ /*             
                    green = round(color.at(row)(1) * 255);
                    blue = round(color.at(row)(2) * 255);
                    break;

                }else {

                    it1 = find(it1+1,v1.end(),x);
                }
            }

            pixel[0] = blue;  // values are stored as BGR and not RGB
            pixel[1] = green;
            pixel[2] = red;

            myfile.write( (char*)pixel, 3 );
        }
        myfile.write( (char*)pad, padSize );
    }

    myfile.close();
}

