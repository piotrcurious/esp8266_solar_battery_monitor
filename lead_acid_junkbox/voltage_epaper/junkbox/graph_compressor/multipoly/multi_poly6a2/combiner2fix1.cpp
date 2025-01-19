// Enhanced determinant calculator for Sylvester matrix
class DeterminantCalculator {
public:
    static float calculateDeterminant(const std::vector<std::vector<float>>& matrix) {
        int n = matrix.size();
        if (n == 0) return 0;
        if (n == 1) return matrix[0][0];
        if (n == 2) return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
        
        float det = 0;
        std::vector<std::vector<float>> submatrix(n - 1, std::vector<float>(n - 1));
        
        for (int x = 0; x < n; x++) {
            // Get cofactor matrix
            for (int i = 1; i < n; i++) {
                int subi = 0;
                for (int j = 0; j < n; j++) {
                    if (j == x) continue;
                    submatrix[i-1][subi] = matrix[i][j];
                    subi++;
                }
            }
            det += (x % 2 == 0 ? 1 : -1) * matrix[0][x] * calculateDeterminant(submatrix);
        }
        return det;
    }
};

// Enhanced projective analysis
class ProjectiveAnalyzer {
public:
    static bool analyzeProjectiveCompletion(const std::vector<float>& coeffs1,
                                          const std::vector<float>& coeffs2,
                                          float boundary) {
        std::vector<float> homogeneous1 = toHomogeneous(coeffs1);
        std::vector<float> homogeneous2 = toHomogeneous(coeffs2);
        
        if (!checkSmoothnessAtInfinity(homogeneous1, homogeneous2)) {
            return false;
        }
        
        return analyzeProjectiveBoundary(homogeneous1, homogeneous2, boundary);
    }
    
private:
    static std::vector<float> toHomogeneous(const std::vector<float>& coeffs) {
        int degree = coeffs.size() - 1;
        std::vector<float> homogeneous(coeffs.size() + 1, 0.0f);
        
        // Convert to homogeneous coordinates by introducing extra variable
        for (int i = 0; i <= degree; i++) {
            homogeneous[i] = coeffs[i] * pow(1.0f, degree - i);
        }
        homogeneous[degree + 1] = 1.0f;
        
        return homogeneous;
    }
    
    static bool checkSmoothnessAtInfinity(const std::vector<float>& h1,
                                         const std::vector<float>& h2) {
        // Check behavior at infinity by analyzing highest degree terms
        if (h1.size() != h2.size()) return false;
        
        // Compare leading coefficients
        float ratio = h1[0] / h2[0];
        for (size_t i = 1; i < 3; i++) {  // Check first few terms
            if (abs(h1[i] / h2[i] - ratio) > 1e-6) {
                return false;
            }
        }
        
        // Check for singular points at infinity
        std::vector<float> derivative1 = computeHomogeneousDerivative(h1);
        std::vector<float> derivative2 = computeHomogeneousDerivative(h2);
        
        return !hasCommonZeroAtInfinity(derivative1, derivative2);
    }
    
    static std::vector<float> computeHomogeneousDerivative(const std::vector<float>& coeffs) {
        std::vector<float> derivative(coeffs.size() - 1);
        for (size_t i = 0; i < coeffs.size() - 1; i++) {
            derivative[i] = coeffs[i] * (coeffs.size() - 1 - i);
        }
        return derivative;
    }
    
    static bool hasCommonZeroAtInfinity(const std::vector<float>& d1,
                                       const std::vector<float>& d2) {
        // Use resultant to check for common zeros
        float resultant = ResultantCalculator::calculateResultant(d1, d2);
        return abs(resultant) < 1e-6;
    }
    
    static bool analyzeProjectiveBoundary(const std::vector<float>& h1,
                                         const std::vector<float>& h2,
                                         float boundary) {
        // Convert boundary point to projective coordinates
        float homBoundary = boundary / sqrt(1 + boundary * boundary);
        float w = 1 / sqrt(1 + boundary * boundary);
        
        // Check continuity in projective space
        float v1 = evaluateHomogeneousPolynomial(h1, homBoundary, w);
        float v2 = evaluateHomogeneousPolynomial(h2, homBoundary, w);
        
        if (abs(v1 - v2) > 1e-6) {
            return false;
        }
        
        // Check derivative continuity in projective space
        std::vector<float> dh1 = computeHomogeneousDerivative(h1);
        std::vector<float> dh2 = computeHomogeneousDerivative(h2);
        
        float dv1 = evaluateHomogeneousPolynomial(dh1, homBoundary, w);
        float dv2 = evaluateHomogeneousPolynomial(dh2, homBoundary, w);
        
        return abs(dv1 - dv2) < 1e-6;
    }
    
    static float evaluateHomogeneousPolynomial(const std::vector<float>& coeffs,
                                              float x, float w) {
        float result = 0;
        int degree = coeffs.size() - 1;
        
        for (int i = 0; i <= degree; i++) {
            result += coeffs[i] * pow(x, degree - i) * pow(w, i);
        }
        
        return result;
    }
};

// Enhanced boundary analysis with topological considerations
class BoundaryAnalyzer {
public:
    struct BoundaryTopology {
        float localDegree;
        bool isRegular;
        std::vector<float> criticalValues;
        float intersectionMultiplicity;
    };
    
    static BoundaryTopology analyzeBoundaryTopology(const std::vector<float>& coeffs1,
                                                   const std::vector<float>& coeffs2,
                                                   float boundary) {
        BoundaryTopology topology;
        
        // Calculate local degree at boundary
        topology.localDegree = calculateLocalDegree(coeffs1, coeffs2, boundary);
        
        // Check regularity at boundary
        topology.isRegular = checkRegularity(coeffs1, coeffs2, boundary);
        
        // Find critical values near boundary
        topology.criticalValues = findCriticalValues(coeffs1, coeffs2, boundary);
        
        // Calculate intersection multiplicity
        topology.intersectionMultiplicity = calculateIntersectionMultiplicity(coeffs1, coeffs2, boundary);
        
        return topology;
    }
    
private:
    static float calculateLocalDegree(const std::vector<float>& c1,
                                    const std::vector<float>& c2,
                                    float boundary) {
        // Calculate order of vanishing at boundary
        float order = 0;
        float epsilon = 1e-6;
        
        for (float t = boundary - epsilon; t <= boundary + epsilon; t += epsilon/10) {
            float v1 = evaluatePolynomial(c1.data(), c1.size(), t);
            float v2 = evaluatePolynomial(c2.data(), c2.size(), t - boundary);
            
            if (abs(v1 - v2) > epsilon) {
                order = log(abs(v1 - v2)) / log(abs(t - boundary));
                break;
            }
        }
        
        return order;
    }
    
    static bool checkRegularity(const std::vector<float>& c1,
                               const std::vector<float>& c2,
                               float boundary) {
        // Check if all derivatives up to order n-1 match at boundary
        int maxOrder = min(c1.size(), c2.size()) - 1;
        std::vector<float> current1 = c1;
        std::vector<float> current2 = c2;
        
        for (int order = 0; order < maxOrder; order++) {
            float v1 = evaluatePolynomial(current1.data(), current1.size(), boundary);
            float v2 = evaluatePolynomial(current2.data(), current2.size(), 0);
            
            if (abs(v1 - v2) > 1e-6) {
                return false;
            }
            
            current1 = computeDerivative(current1);
            current2 = computeDerivative(current2);
        }
        
        return true;
    }
    
    static std::vector<float> findCriticalValues(const std::vector<float>& c1,
                                                const std::vector<float>& c2,
                                                float boundary) {
        std::vector<float> criticalValues;
        float epsilon = 1e-6;
        
        // Find zeros of derivatives near boundary
        std::vector<float> d1 = computeDerivative(c1);
        std::vector<float> d2 = computeDerivative(c2);
        
        for (float t = boundary - 0.1f; t <= boundary + 0.1f; t += epsilon) {
            float deriv1 = evaluatePolynomial(d1.data(), d1.size(), t);
            float deriv2 = evaluatePolynomial(d2.data(), d2.size(), t - boundary);
            
            if (abs(deriv1) < epsilon || abs(deriv2) < epsilon) {
                float value = evaluatePolynomial(c1.data(), c1.size(), t);
                criticalValues.push_back(value);
            }
        }
        
        return criticalValues;
    }
    
    static float calculateIntersectionMultiplicity(const std::vector<float>& c1,
                                                 const std::vector<float>& c2,
                                                 float boundary) {
        // Use Bezout's theorem to calculate intersection multiplicity
        float multiplicity = 0;
        std::vector<float> current1 = c1;
        std::vector<float> current2 = c2;
        
        while (true) {
            float v1 = evaluatePolynomial(current1.data(), current1.size(), boundary);
            float v2 = evaluatePolynomial(current2.data(), current2.size(), 0);
            
            if (abs(v1 - v2) > 1e-6) {
                break;
            }
            
            multiplicity += 1;
            current1 = computeDerivative(current1);
            current2 = computeDerivative(current2);
            
            if (current1.empty() || current2.empty()) {
                break;
            }
        }
        
        return multiplicity;
    }
    
    static std::vector<float> computeDerivative(const std::vector<float>& coeffs) {
        if (coeffs.size() <= 1) return std::vector<float>();
        
        std::vector<float> derivative(coeffs.size() - 1);
        for (size_t i = 1; i < coeffs.size(); i++) {
            derivative[i-1] = coeffs[i] * i;
        }
        return derivative;
    }
};

// The main combinePolynomials function remains the same but now uses these enhanced classes
// Add these lines before the combination decision:

BoundaryAnalyzer::BoundaryTopology topology = BoundaryAnalyzer::analyzeBoundaryTopology(
    coeffs1, coeffs2, oldest.timeDeltas[i]);

// Update combination criteria to include topology analysis:
bool canCombine = 
    projectivelySmooth &&
    !features1.hasRealSingularity &&
    !features2.hasRealSingularity &&
    features1.discriminant * features2.discriminant > 0 &&
    features1.inflectionPoints.size() + features2.inflectionPoints.size() <= 3 &&
    topology.isRegular &&
    topology.intersectionMultiplicity >= 1;  // Ensures at least C0 continuity
