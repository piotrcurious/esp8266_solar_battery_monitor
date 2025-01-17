#include <BasicLinearAlgebra.h>    // Use BasicLinearAlgebra instead of Eigen for ESP32

struct DomainPolynomial {
    float* coefficients;  // coefficients from lowest to highest degree
    uint8_t degree;      // polynomial degree
    float x_min;         // domain start
    float x_max;         // domain end
};

// Function to evaluate polynomial at given x
float evaluatePolynomial(const DomainPolynomial& poly, float x) {
    float result = 0;
    float x_power = 1;
    
    for (uint8_t i = 0; i <= poly.degree; i++) {
        result += poly.coefficients[i] * x_power;
        x_power *= x;
    }
    return result;
}

// Get polynomial value based on domain
float getValueFromDomains(const DomainPolynomial& poly1, const DomainPolynomial& poly2, float x) {
    if (x >= poly1.x_min && x <= poly1.x_max) {
        return evaluatePolynomial(poly1, x);
    } else if (x >= poly2.x_min && x <= poly2.x_max) {
        return evaluatePolynomial(poly2, x);
    }
    return 0; // Out of both domains
}

// Main function to fit combined polynomial
DomainPolynomial fitCombinedPolynomial(
    const DomainPolynomial& poly1,
    const DomainPolynomial& poly2,
    uint8_t resultDegree = 3     // degree of resulting polynomial
) {
    // Verify domains are adjacent
    if (abs(poly1.x_max - poly2.x_min) > 0.0001f) {
        Serial.println("Error: Domains must be adjacent");
        return DomainPolynomial{nullptr, 0, 0, 0};
    }

    // Number of equations to generate (2 per coefficient for better fit)
    const uint8_t numEquations = (resultDegree + 1) * 2;
    
    // Create matrices for normal equations
    BLA::Matrix<numEquations, resultDegree + 1> A;
    BLA::Matrix<numEquations, 1> b;
    
    // Generate equations using evenly spaced points across both domains
    float total_range = poly2.x_max - poly1.x_min;
    float dx = total_range / (numEquations - 1);
    
    for (uint8_t i = 0; i < numEquations; i++) {
        float x = poly1.x_min + i * dx;
        float x_power = 1.0f;
        
        // Fill A matrix row
        for (uint8_t j = 0; j <= resultDegree; j++) {
            A(i, j) = x_power;
            x_power *= x;
        }
        
        // Fill b vector with actual polynomial value
        b(i, 0) = getValueFromDomains(poly1, poly2, x);
    }
    
    // Solve normal equations using transpose multiplication
    BLA::Matrix<resultDegree + 1, resultDegree + 1> ATA = ~A * A;
    BLA::Matrix<resultDegree + 1, 1> ATb = ~A * b;
    
    // Gaussian elimination to solve ATA * x = ATb
    BLA::Matrix<resultDegree + 1, 1> solution;
    bool solved = GaussianElimination(ATA, ATb, solution);
    
    if (!solved) {
        Serial.println("Error: Failed to solve system");
        return DomainPolynomial{nullptr, 0, 0, 0};
    }
    
    // Create result polynomial
    DomainPolynomial result;
    result.degree = resultDegree;
    result.x_min = poly1.x_min;
    result.x_max = poly2.x_max;
    result.coefficients = new float[resultDegree + 1];
    
    for (uint8_t i = 0; i <= resultDegree; i++) {
        result.coefficients[i] = solution(i, 0);
    }
    
    return result;
}

// Gaussian elimination solver
bool GaussianElimination(BLA::Matrix<>& A, BLA::Matrix<>& b, BLA::Matrix<>& x) {
    int n = A.Rows;
    
    // Forward elimination
    for (int i = 0; i < n; i++) {
        // Find pivot
        float maxEl = abs(A(i,i));
        int maxRow = i;
        for (int k = i + 1; k < n; k++) {
            if (abs(A(k,i)) > maxEl) {
                maxEl = abs(A(k,i));
                maxRow = k;
            }
        }
        
        if (maxEl < 0.00001f) return false; // Matrix is singular
        
        // Swap maximum row with current row
        if (maxRow != i) {
            for (int k = i; k < n; k++) {
                float tmp = A(i,k);
                A(i,k) = A(maxRow,k);
                A(maxRow,k) = tmp;
            }
            float tmp = b(i,0);
            b(i,0) = b(maxRow,0);
            b(maxRow,0) = tmp;
        }
        
        // Eliminate column
        for (int k = i + 1; k < n; k++) {
            float c = -A(k,i) / A(i,i);
            for (int j = i; j < n; j++) {
                if (i == j)
                    A(k,j) = 0;
                else
                    A(k,j) += c * A(i,j);
            }
            b(k,0) += c * b(i,0);
        }
    }
    
    // Back substitution
    for (int i = n - 1; i >= 0; i--) {
        x(i,0) = b(i,0);
        for (int k = i + 1; k < n; k++) {
            x(i,0) -= A(i,k) * x(k,0);
        }
        x(i,0) /= A(i,i);
    }
    
    return true;
}

// Example usage:
void setup() {
    Serial.begin(115200);
    
    // Example polynomials
    float coeffs1[] = {1.0, 2.0, 1.0}; // x^2 + 2x + 1
    DomainPolynomial poly1 = {
        coeffs1,
        2,    // degree
        0.0,  // x_min
        2.0   // x_max
    };
    
    float coeffs2[] = {2.0, -1.0, 0.5}; // 0.5x^2 - x + 2
    DomainPolynomial poly2 = {
        coeffs2,
        2,    // degree
        2.0,  // x_min
        4.0   // x_max
    };
    
    // Fit combined polynomial
    DomainPolynomial result = fitCombinedPolynomial(poly1, poly2);
    
    if (result.coefficients != nullptr) {
        // Print results
        Serial.println("Fitted polynomial coefficients:");
        for (uint8_t i = 0; i <= result.degree; i++) {
            Serial.print("x^");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(result.coefficients[i]);
        }
        
        // Clean up
        delete[] result.coefficients;
    }
}

void loop() {
    // Empty loop
}
