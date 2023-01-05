#include "matplotlib-cpp/matplotlibcpp.h"
#include <cmath>
#include <Eigen/Eigen> 
#include<iostream>
using namespace std;
#include <vector>
#include <ros/ros.h>


namespace plt = matplotlibcpp;

class figure
{
private:
    std::vector<Eigen::VectorXd> results;
    size_t fig_num;
    int fig_columns = 3;
public:
    figure(){         
    };
    ~figure(){};
    void setResults(std::vector<Eigen::VectorXd> results_){results = results_;};
    void plot(std::map<int, std::string> ylabel_);
};
