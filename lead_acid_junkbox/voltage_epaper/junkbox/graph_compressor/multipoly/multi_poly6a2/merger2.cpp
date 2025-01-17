#include <BasicLinearAlgebra.h>

struct DomainPolynomial {
    float* coefficients;
    uint8_t degree;
    float x_min;         // original domain start
    float x_max;         // original domain end
    float domain_length; // cached for efficiency
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

// Map an x value from the combined domain to the polynomial's original domain
float mapToDomain(float x, const DomainPolynomial& poly, float eval_start) {
    // Calculate how many domain lengths we are from the evaluation start
    float domains_offset = (x - eval_start) / poly.domain_length;
    // Map back to original domain
    return poly.x_min + (domains_offset - floor(domains_offset)) * poly.domain_length;
}

// Get polynomial value based on domains, handling wrapping and overlaps
float getValueFromDomains(const DomainPolynomial& poly1, const DomainPolynomial& poly2, float x) {
    float value = 0;
    int count = 0;
    
    // Check if x is in range of poly1 (including wrapped domains)
    if (x >= poly1.x_min) {
        float mapped_x = mapToDomain(x, poly1, poly1.x_min);
        value += evaluatePolynomial(poly1, mapped_x);
        count++;
    }
    
    // Check if x is in range of poly2 (including wrapped domains)
    if (x >= poly2.x_min) {
        float mapped_x = mapToDomain(x, poly2, poly2.x_min);
        value += evaluatePolynomial(poly2, mapped_x);
        count++;
    }
    
    return count > 0 ? value / count : 0;
}

// Main function to fit combined polynomial
DomainPolynomial fitCombinedPolynomial(
    DomainPolynomial poly1,
    DomainPolynomial poly2,
    uint8_t resultDegree = 3
) {
    // Calculate and cache domain lengths
    poly1.domain_length = poly1.x_max - poly1.x_min;
    poly2.domain_length = poly2.x_max - poly2.x_min;
    
    // Find combined domain range
    float combined_min = min(poly1.x_min, poly2.x_min);
    float combined_max = max(poly1.x_max + poly1.domain_length, 
                           poly2.x_max + poly2.domain_length);
    
    // Number of equations to generate (2 per coefficient for better fit)
    const uint8_t numEquations = (resultDegree + 1) * 2;
    
    // Create matrices for normal equations
    BLA::Matrix<numEquations, resultDegree + 1> A;
    BLA::Matrix<numEquations, 1> b;
    
    // Generate equations using evenly spaced points across combined domain
    float total_range = combined_max - combined_min;
    float dx = total_range / (numEquations - 1);
    
    for (uint8_t i = 0; i < numEquations; i++) {
        float x = combined_min + i * dx;
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
        return DomainPolynomial{nullptr, 0, 0, 0, 0};
    }
    
    // Create result polynomial
    DomainPolynomial result;
    result.degree = resultDegree;
    result.x_min = combined_min;
    result.x_max = combined_max;
    result.domain_length = total_range;
    result.coefficients = new float[resultDegree + 1];
    
    for (uint8_t i = 0; i <= resultDegree; i++) {
        result.coefficients[i] = solution(i, 0);
    }
    
    return result;
}

// Gaussian elimination solver (unchanged)
bool GaussianElimination(BLA::Matrix<>& A, BLA::Matrix<>& b, BLA::Matrix<>& x) {
    int n = A.Rows;
    
    // Forward elimination
    for (int i = 0; i < n; i++) {
        float maxEl = abs(A(i,i));
        int maxRow = i;
        for (int k = i + 1; k < n; k++) {
            if (abs(A(k,i)) > maxEl) {
                maxEl = abs(A(k,i));
                maxRow = k;
            }
        }
        
        if (maxEl < 0.00001f) return false;
        
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
    
    // Example: Two polynomials with same domain 0-1
    float coeffs1[] = {1.0, 2.0, 1.0}; // x^2 + 2x + 1
    DomainPolynomial poly1 = {
        coeffs1,
        2,    // degree
        0.0,  // x_min
        1.0,  // x_max
        0.0   // domain_length (will be calculated)
    };
    
    float coeffs2[] = {2.0, -1.0, 0.5}; // 0.5x^2 - x + 2
    DomainPolynomial poly2 = {
        coeffs2,
        2,    // degree
        0.0,  // x_min
        1.0,  // x_max
        0.0   // domain_length (will be calculated)
    };
    
    // Fit combined polynomial
    DomainPolynomial result = fitCombinedPolynomial(poly1, poly2);
    
    if (result.coefficients != nullptr) {
        Serial.println("Fitted polynomial coefficients:");
        for (uint8_t i = 0; i <= result.degree; i++) {
            Serial.print("x^");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(result.coefficients[i]);
        }
        Serial.print("Domain: [");
        Serial.print(result.x_min);
        Serial.print(", ");
        Serial.print(result.x_max);
        Serial.println("]");
        
        // Test some values
        float test_x[] = {0.5, 1.5, 2.5};
        for (int i = 0; i < 3; i++) {
            Serial.print("Value at x=");
            Serial.print(test_x[i]);
            Serial.print(": ");
            Serial.println(getValueFromDomains(poly1, poly2, test_x[i]));
        }
        
        delete[] result.coefficients;
    }
}

void loop() {
    // Empty loop
}
