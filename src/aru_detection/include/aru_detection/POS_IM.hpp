#include <deque>
#include <vector>
#include <numeric>

class POINTS {
public:
    POINTS() : n_(10.0), alpha_(0.5), x_estA_(0.0), max_jump_(200.0) {
        queue_.resize(3, std::deque<double>());
    }

    // Moving Average
    double ma_get(double x, int num_state_var) {

        auto& q = queue_[num_state_var];

        if (!q.empty() && std::abs(x - q.back()) > max_jump_) {
            q.clear();  // RESET
        }

        if ((float)q.size() >= n_) {
            q.pop_front();
        }
        q.push_back(x);
                                            
        double sum = std::accumulate(q.begin(), q.end(), 0.0);
        return sum / q.size();
    }

private:
    int n_;
    double alpha_;
    double x_estA_;
    double max_jump_;
    std::vector<std::deque<double>> queue_;
};
