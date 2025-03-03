int examineExample(size_t i2, size_t n, std::vector<double>& alpha, const std::vector<float>& y,
                 std::vector<double>& f, double epsilon, double C,
                 const std::vector<std::vector<double>>& K, double tol, double& b);

int optimizePair(size_t i1, size_t i2, size_t n, std::vector<double>& alpha, const std::vector<float>& y,
                std::vector<double>& f, double epsilon, double C,
                const std::vector<std::vector<double>>& K, double tol, double& b);

std::vector<double> denormalizeCoefficients(const std::vector<double>& w, double x_min, double x_range, int degree);

double binomialCoefficient(int n, int k);
