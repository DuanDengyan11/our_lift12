#include <figure/figure.h>
using namespace std;

void figure::plot(std::map<int, std::string> ylabel_)
{
    plt::figure(1);
    
    size_t iter_num = results.size();
    size_t variable_num = results[0].size(); 
    fig_num = variable_num - 1;

    // columns = 3
    int fig_rows = ceil(fig_num / fig_columns);
    // plt::figure_size(1800, 1200);

    std::vector<std::vector<double>> datas;
    std::vector<double> data;
    
    for(size_t j = 0; j < variable_num; j++)
    {
        data.clear();
        for (size_t i = 0; i < iter_num; i++)
        {
            data.push_back(results[i](j));
        }
        datas.push_back(data);
    }

    for (size_t i = 0; i < fig_num; i++)
    {
        plt::subplot(2, 3, i+1);
        plt::plot( datas[0], datas[i+1]);
        plt::ylabel(ylabel_[i]);
    }
    
    std::map<std::string, double> m = {{"hspace", 0.3}, {"wspace", 0.45}};
    plt::subplots_adjust(m);
    plt::pause(0.001);
    // plt::legend();
}

int main()
{
    // Prepare data.
    int n = 5000;
    Eigen::VectorXd data(3);
    std::vector<Eigen::VectorXd> datas;
    for(int i=0; i<n; ++i) {
        data[0] = i*i;
        data[1] = sin(2*M_PI*i/360.0);
        data[2] = log(i);
        datas.push_back(data);
    }
    figure test;
              std::map<int, std::string> ylabel_ = {{0, "smooth_cost"}, 
          {1, "feasibility"},
          {2, "uav obstacle"},
          {3, "uav swarm"}, 
          {4, "cable obstacle"}};
    test.setResults(datas);

    while (1)
    {
    test.plot(ylabel_);
    plt::pause(1);
    
    // plt::figure(2);
    // test.plot(ylabel_);
    // plt::pause(0.001);
    // plt::clf();
    // plt::figure();
    // plt::show();
    }
    

}