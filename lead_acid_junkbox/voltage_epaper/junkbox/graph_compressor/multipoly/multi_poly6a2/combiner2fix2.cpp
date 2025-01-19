// Optimal split point finder using geometric and analytical methods
class OptimalSplitFinder {
public:
    struct SplitQuality {
        float splitPoint;
        float errorMetric;
        float continuityMetric;
        float curvatureMetric;
        float totalQuality;
    };
    
    static SplitQuality findOptimalSplit(const std::vector<float>& coeffs1,
                                        const std::vector<float>& coeffs2,
                                        float t1, float t2,
                                        float resolution,
                                        const std::vector<float>& originalValues) {
        // Initialize quality metrics
        SplitQuality bestSplit;
        bestSplit.errorMetric = INFINITY;
        bestSplit.totalQuality = -INFINITY;
        
        // Calculate initial approximation using geometric features
        float initialGuess = findInitialSplitGuess(coeffs1, coeffs2, t1, t2, resolution);
        
        // Refine split point using gradient descent
        std::vector<float> searchPoints = generateSearchPoints(initialGuess, t1, t2, resolution);
        
        for (float candidateSplit : searchPoints) {
            SplitQuality quality = evaluateSplitQuality(coeffs1, coeffs2, candidateSplit, 
                                                      t1, t2, resolution, originalValues);
            
            if (quality.totalQuality > bestSplit.totalQuality) {
                bestSplit = quality;
            }
        }
        
        // Fine-tune using Newton's method
        bestSplit = refineSplitPoint(coeffs1, coeffs2, bestSplit.splitPoint, 
                                   t1, t2, resolution, originalValues);
        
        return bestSplit;
    }

private:
    static float findInitialSplitGuess(const std::vector<float>& coeffs1,
                                      const std::vector<float>& coeffs2,
                                      float t1, float t2,
                                      float resolution) {
        std::vector<float> criticalPoints;
        
        // Find inflection points
        auto derivatives1 = computeDerivatives(coeffs1, 3);
        auto derivatives2 = computeDerivatives(coeffs2, 3);
        
        // Analyze first polynomial
        for (float t = 0; t <= t1; t += resolution) {
            if (isInflectionPoint(derivatives1, t)) {
                criticalPoints.push_back(t);
            }
        }
        
        // Analyze second polynomial
        for (float t = 0; t <= t2; t += resolution) {
            if (isInflectionPoint(derivatives2, t)) {
                criticalPoints.push_back(t + t1);
            }
        }
        
        // If no inflection points found, use curvature analysis
        if (criticalPoints.empty()) {
            return findMaxCurvaturePoint(coeffs1, coeffs2, t1, t2, resolution);
        }
        
        // Return the critical point closest to the middle
        float midPoint = (t1 + t2) / 2;
        return *std::min_element(criticalPoints.begin(), criticalPoints.end(),
            [midPoint](float a, float b) {
                return std::abs(a - midPoint) < std::abs(b - midPoint);
            });
    }
    
    static std::vector<std::vector<float>> computeDerivatives(const std::vector<float>& coeffs,
                                                            int order) {
        std::vector<std::vector<float>> derivatives;
        derivatives.push_back(coeffs);
        
        for (int i = 1; i <= order; i++) {
            std::vector<float> derivative(coeffs.size() - i);
            for (size_t j = i; j < coeffs.size(); j++) {
                derivative[j-i] = coeffs[j] * factorial(j) / factorial(j - i);
            }
            derivatives.push_back(derivative);
        }
        
        return derivatives;
    }
    
    static bool isInflectionPoint(const std::vector<std::vector<float>>& derivatives,
                                float t) {
        float secondDeriv = evaluatePolynomial(derivatives[2].data(), 
                                             derivatives[2].size(), t);
        float thirdDeriv = evaluatePolynomial(derivatives[3].data(), 
                                            derivatives[3].size(), t);
        
        return std::abs(secondDeriv) < 1e-6 && std::abs(thirdDeriv) > 1e-6;
    }
    
    static float findMaxCurvaturePoint(const std::vector<float>& coeffs1,
                                     const std::vector<float>& coeffs2,
                                     float t1, float t2,
                                     float resolution) {
        float maxCurvature = 0;
        float maxCurvaturePoint = t1;
        
        auto derivatives1 = computeDerivatives(coeffs1, 2);
        auto derivatives2 = computeDerivatives(coeffs2, 2);
        
        // Analyze curvature along both polynomials
        for (float t = 0; t <= t1 + t2; t += resolution) {
            float curvature;
            if (t <= t1) {
                curvature = calculateCurvature(derivatives1, t);
            } else {
                curvature = calculateCurvature(derivatives2, t - t1);
            }
            
            if (std::abs(curvature) > maxCurvature) {
                maxCurvature = std::abs(curvature);
                maxCurvaturePoint = t;
            }
        }
        
        return maxCurvaturePoint;
    }
    
    static float calculateCurvature(const std::vector<std::vector<float>>& derivatives,
                                  float t) {
        float firstDeriv = evaluatePolynomial(derivatives[1].data(), 
                                            derivatives[1].size(), t);
        float secondDeriv = evaluatePolynomial(derivatives[2].data(), 
                                             derivatives[2].size(), t);
        
        return std::abs(secondDeriv) / std::pow(1 + firstDeriv * firstDeriv, 1.5);
    }
    
    static std::vector<float> generateSearchPoints(float initialGuess,
                                                 float t1, float t2,
                                                 float resolution) {
        std::vector<float> searchPoints;
        float searchRadius = (t2 - t1) * 0.2f; // 20% of total interval
        
        // Generate points with decreasing density around initial guess
        for (float offset = -searchRadius; offset <= searchRadius; offset += resolution) {
            float candidatePoint = initialGuess + offset;
            if (candidatePoint > t1 * 0.5f && candidatePoint < (t1 + t2) * 0.95f) {
                searchPoints.push_back(candidatePoint);
            }
        }
        
        return searchPoints;
    }
    
    static SplitQuality evaluateSplitQuality(const std::vector<float>& coeffs1,
                                           const std::vector<float>& coeffs2,
                                           float splitPoint,
                                           float t1, float t2,
                                           float resolution,
                                           const std::vector<float>& originalValues) {
        SplitQuality quality;
        quality.splitPoint = splitPoint;
        
        // Generate data for fitting
        std::vector<float> timestamps1, values1;
        std::vector<float> timestamps2, values2;
        
        // First segment
        for (float t = 0; t <= splitPoint; t += resolution) {
            timestamps1.push_back(t);
            if (t <= t1) {
                values1.push_back(evaluatePolynomial(coeffs1.data(), coeffs1.size(), t));
            } else {
                values1.push_back(evaluatePolynomial(coeffs2.data(), coeffs2.size(), t - t1));
            }
        }
        
        // Second segment
        for (float t = splitPoint; t <= t1 + t2; t += resolution) {
            timestamps2.push_back(t - splitPoint);
            values2.push_back(evaluatePolynomial(coeffs2.data(), coeffs2.size(), t - t1));
        }
        
        // Fit polynomials to both segments
        AdvancedPolynomialFitter fitter;
        std::vector<float> fitCoeffs1 = fitter.fitPolynomial(timestamps1, values1, 5,
                                                            AdvancedPolynomialFitter::NONE);
        std::vector<float> fitCoeffs2 = fitter.fitPolynomial(timestamps2, values2, 5,
                                                            AdvancedPolynomialFitter::NONE);
        
        // Calculate error metric
        quality.errorMetric = calculateErrorMetric(fitCoeffs1, fitCoeffs2, splitPoint,
                                                 t1, t2, resolution, originalValues);
        
        // Calculate continuity metric
        quality.continuityMetric = calculateContinuityMetric(fitCoeffs1, fitCoeffs2,
                                                           splitPoint);
        
        // Calculate curvature metric
        quality.curvatureMetric = calculateCurvatureMetric(fitCoeffs1, fitCoeffs2,
                                                         splitPoint);
        
        // Calculate total quality
        quality.totalQuality = calculateTotalQuality(quality);
        
        return quality;
    }
    
    static float calculateErrorMetric(const std::vector<float>& coeffs1,
                                    const std::vector<float>& coeffs2,
                                    float splitPoint,
                                    float t1, float t2,
                                    float resolution,
                                    const std::vector<float>& originalValues) {
        float totalError = 0;
        int count = 0;
        
        // Calculate error for first segment
        for (float t = 0; t <= splitPoint; t += resolution) {
            float fitted = evaluatePolynomial(coeffs1.data(), coeffs1.size(), t);
            float original = originalValues[count++];
            totalError += (fitted - original) * (fitted - original);
        }
        
        // Calculate error for second segment
        for (float t = 0; t <= t2 - (splitPoint - t1); t += resolution) {
            float fitted = evaluatePolynomial(coeffs2.data(), coeffs2.size(), t);
            float original = originalValues[count++];
            totalError += (fitted - original) * (fitted - original);
        }
        
        return sqrt(totalError / count);
    }
    
    static float calculateContinuityMetric(const std::vector<float>& coeffs1,
                                         const std::vector<float>& coeffs2,
                                         float splitPoint) {
        float metric = 0;
        
        // Check continuity up to second derivative
        for (int order = 0; order <= 2; order++) {
            std::vector<float> d1 = nthDerivativeCoefficients(coeffs1, order);
            std::vector<float> d2 = nthDerivativeCoefficients(coeffs2, order);
            
            float v1 = evaluatePolynomial(d1.data(), d1.size(), splitPoint);
            float v2 = evaluatePolynomial(d2.data(), d2.size(), 0);
            
            metric += abs(v1 - v2) * pow(10.0f, -order); // Weight higher derivatives less
        }
        
        return metric;
    }
    
    static float calculateCurvatureMetric(const std::vector<float>& coeffs1,
                                        const std::vector<float>& coeffs2,
                                        float splitPoint) {
        auto d1 = computeDerivatives(coeffs1, 2);
        auto d2 = computeDerivatives(coeffs2, 2);
        
        float c1 = calculateCurvature(d1, splitPoint);
        float c2 = calculateCurvature(d2, 0);
        
        return abs(c1 - c2);
    }
    
    static float calculateTotalQuality(const SplitQuality& quality) {
        // Weighted sum of individual metrics
        return -quality.errorMetric * 0.4f
               - quality.continuityMetric * 0.4f
               - quality.curvatureMetric * 0.2f;
    }
    
    static SplitQuality refineSplitPoint(const std::vector<float>& coeffs1,
                                       const std::vector<float>& coeffs2,
                                       float initialSplit,
                                       float t1, float t2,
                                       float resolution,
                                       const std::vector<float>& originalValues) {
        const float epsilon = resolution * 0.1f;
        const int maxIterations = 10;
        float currentSplit = initialSplit;
        
        for (int i = 0; i < maxIterations; i++) {
            // Calculate quality at current point and nearby points
            SplitQuality current = evaluateSplitQuality(coeffs1, coeffs2, currentSplit,
                                                      t1, t2, resolution, originalValues);
            SplitQuality left = evaluateSplitQuality(coeffs1, coeffs2, currentSplit - epsilon,
                                                   t1, t2, resolution, originalValues);
            SplitQuality right = evaluateSplitQuality(coeffs1, coeffs2, currentSplit + epsilon,
                                                    t1, t2, resolution, originalValues);
            
            // Approximate derivatives
            float firstDeriv = (right.totalQuality - left.totalQuality) / (2 * epsilon);
            float secondDeriv = (right.totalQuality - 2 * current.totalQuality + left.totalQuality) 
                               / (epsilon * epsilon);
            
            // Newton's method step
            float step = -firstDeriv / (secondDeriv + 1e-6f);
            
            // Update split point
            float newSplit = currentSplit + step;
            
            // Check for convergence
            if (abs(step) < epsilon * 0.1f) {
                break;
            }
            
            // Ensure split point stays within bounds
            currentSplit = std::max(t1 * 0.5f, std::min(newSplit, (t1 + t2) * 0.95f));
        }
        
        return evaluateSplitQuality(coeffs1, coeffs2, currentSplit,
                                  t1, t2, resolution, originalValues);
    }
    
    static std::vector<float> nthDerivativeCoefficients(const std::vector<float>& coeffs,
                                                      int n) {
        std::vector<float> result = coeffs;
        for (int i = 0; i < n; i++) {
            for (size_t j = 1; j < result.size(); j++) {
                result[j-1] = result[j] * j;
            }
            result.pop_back();
        }
        return result;
    }
    
    static int factorial(int n) {
        if (n <= 1) return 1;
        return n * factorial(n-1);
    }
};

// Update the main combinePolynomials function to use the new split finder
// Add before the splitting logic:

OptimalSplitFinder::SplitQuality splitQuality = OptimalSplitFinder::fin</antArtifact>
