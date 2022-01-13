#include <cmath>
#include <iostream>
#include <vector>
#include "Eigen/Eigen"
using namespace Eigen;

//##################################
MatrixXd AppendData(MatrixXd &A, MatrixXd &B)
{
    MatrixXd D(A.rows() + B.rows(), A.cols());
    D << A, B;
    return D;
}
//##################################
float distance(std::vector<float> xy1, std::vector<float> xy2)
{
    return sqrt((xy1[0] - xy2[0]) * (xy1[0] - xy2[0]) + (xy1[1] - xy2[1]) * (xy1[1] - xy2[1]));
}
//##################################
std::vector<MatrixXd> Segment(float origin_data[][2], int size)
{
    std::vector<std::vector<float>> data;
    std::vector<MatrixXd> Returndata;

    for (int i = 0; i < size; i++) {
        if (((origin_data[i][0] != 0) || (origin_data[i][1] != 0)) && (std::isnormal(origin_data[i][0]) && std::isnormal(origin_data[i][1]))) {
            data.push_back({ origin_data[i][0], origin_data[i][1] });
        }
    }
    float threshold = 0.4;
    int validsize = data.size(), ReturnEndIDX = 0;
    bool one_is_end = distance(data[0], data[validsize - 1]) < threshold;

    MatrixXd tmp(1, 2);
    tmp << data[0][0], data[0][1];
    Returndata.push_back(tmp);
    for (int i = 1; i < validsize; i++) {
        tmp << data[i][0], data[i][1];
        if (distance(data[i - 1], data[i]) < threshold)
            Returndata[ReturnEndIDX] = AppendData(Returndata[ReturnEndIDX], tmp);

        else {
            ReturnEndIDX++;
            Returndata.push_back(tmp);
        }
    }

    if (one_is_end) {
        Returndata[0] = AppendData(Returndata[ReturnEndIDX], Returndata[0]);
        Returndata.pop_back();
    }
    return Returndata;
}
//##################################
struct s_r {
    float s;
    float r;
};
//##################################
struct s_r circle_fit(MatrixXd &data)
{
    MatrixXd A(data.rows(), data.cols() + 1);
    MatrixXd b(data.rows(), 1);
    b << (-1 * data.array().square().matrix()) * MatrixXd::Ones(data.cols(), 1);
    A << -2 * data, MatrixXd::Ones(data.rows(), 1);
    MatrixXd solution = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);

    MatrixXd center(1, data.cols());
    for (int i = 0; i < data.cols(); i++) {
        center(0, i) = solution(i, 0);
    }

    float r = solution(data.cols(), 0), s = 0;
    r = sqrt(pow(center.norm(), 2) - r);
    for (int i = 0; i < data.rows(); i++) {
        s = s + pow((data.row(i) - center).norm() - r, 2);
    }
    s = sqrt(s);
    struct s_r Returndata;
    Returndata.s = s;
    Returndata.r = r;
    return Returndata;
}
//##################################
int main()
{
    float x[] = { 1, 0.996194698091746, 0.984807753012208, 0.965925826289068, 0.939692620785908, 0.906307787036650, 0.866025403784439, 0.819152044288992, 0.766044443118978, 0.707106781186548, 10, 10.0555555555556, 10.1111111111111, 10.1666666666667, 10.2222222222222, 10.2777777777778, 10.3333333333333, 10.3888888888889, 10.4444444444444, 10.5000000000000 };
    float y[] = { 0, 0.0871557427476582, 0.173648177666930, 0.258819045102521, 0.342020143325669, 0.422618261740699, 0.500000000000000, 0.573576436351046, 0.642787609686539, 0.707106781186548, 10, 10.0555555555556, 10.1111111111111, 10.1666666666667, 10.2222222222222, 10.2777777777778, 10.3333333333333, 10.3888888888889, 10.4444444444444, 10.5000000000000 };
    float xy[20][2];
    for (int i = 0; i < 20; i++) {
        xy[i][0] = x[i];
        xy[i][1] = y[i];
    }
    std::vector<MatrixXd> Seg_data = Segment(xy, 20);
    for (int i = 0; i < Seg_data.size(); i++) {
        std::cout << i + 1 << "-th segment\n"
                  << Seg_data[i] << '\n';
        struct s_r s_and_r = circle_fit(Seg_data[i]);
        std::cout << "s :" << s_and_r.s << " , r :" << s_and_r.r << "\n\n";
    }

    return 0;
}
