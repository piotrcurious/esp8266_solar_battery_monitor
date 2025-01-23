#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

// Function to perform total least squares polynomial fit
void tlsPolynomialFit(const float* x_data, const float* y_data, int data_size, int poly_order, std::vector<float>& coeffs) {
    // Prepare the data matrix A and observation vector B
    MatrixXf A(data_size, poly_order + 1);
    VectorXf B(data_size);

    for (int i = 0; i < data_size; i++) {
        float xi = x_data[i];
        B(i) = y_data[i];
        for (int j = 0; j <= poly_order; j++) {
            A(i, j) = pow(xi, j);  // Fill A with powers of x
        }
    }

    // Concatenate A and B to form the augmented matrix
    MatrixXf augmented(data_size, poly_order + 2);
    augmented << A, B;

    // Perform SVD to solve the total least squares problem
    JacobiSVD<MatrixXf> svd(augmented, ComputeFullV);
    MatrixXf V = svd.matrixV();

    // Last column of V corresponds to the null-space solution
    VectorXf solution = V.col(V.cols() - 1);

    // Extract polynomial coefficients by normalizing the last element to 1
    coeffs.clear();
    for (int i = 0; i <= poly_order; i++) {
        coeffs.push_back(-solution(i) / solution(poly_order + 1));
    }
}

void setup() {
    Serial.begin(115200);

    // Example data (x and y values)
    float x_vals[] = {1.0, 2.0, 3.0, 4.0, 5.0};
    float y_vals[] = {2.2, 2.8, 3.6, 4.5, 5.1};
    int num_points = 5;
    int polynomial_order = 2;  // Quadratic fit

    std::vector<float> coefficients;

    // Perform TLS polynomial fit
    tlsPolynomialFit(x_vals, y_vals, num_points, polynomial_order, coefficients);

    // Print the fitted coefficients
    Serial.println("Polynomial coefficients:");
    for (size_t i = 0; i < coefficients.size(); i++) {
        Serial.print("a[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.println(coefficients[i], 6);
    }
}

void loop() {
    // Nothing to do in the loop
}
