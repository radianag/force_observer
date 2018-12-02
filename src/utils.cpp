//
// Created by radian on 11/17/18.
//
#include "utils.h"

Eigen::MatrixXd readCSV(std::string file, int rows, int cols) {

    std::ifstream in(file);

    std::string line;

    int row = 0;
    int col = 0;

    Eigen::MatrixXd res = Eigen::MatrixXd(rows, cols);

    if (in.is_open()) {

        while (std::getline(in, line)) {

            char *ptr = (char *) line.c_str();
            int len = line.length();

            col = 0;

            char *start = ptr;
            for (int i = 0; i < len; i++) {

                if (ptr[i] == ',') {
                    res(row, col++) = atof(start);
                    start = ptr + i + 1;
                }
            }
            res(row, col) = atof(start);

            row++;
        }

        in.close();
    }
    return res;
}

Eigen::Matrix4d modified_dh(const double alpha, const double a, const double d, const double theta ){
   Eigen::Matrix4d R;

   R(0, 0) = cos(theta);
   R(0, 1) = -sin(theta);
   R(0, 2) = 0;
   R(0, 3) = a;
   R(1, 0) = sin(theta)*cos(alpha);
   R(1, 1) = cos(theta)*cos(alpha);
   R(1, 2) = -sin(alpha);
   R(1, 3) = -sin(alpha) * d;
   R(2, 0) = sin(theta) * sin(alpha);
   R(2, 1) = cos(theta) * sin(alpha);
   R(2, 2) = cos(alpha);
   R(2, 3) = cos(alpha) * d;
   R(3, 0) = 0;
   R(3, 1) = 0;
   R(3, 2) = 0;
   R(3, 3) = 1;

   return R;
}

