#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

// Kernel base class for different regression techniques
class RegressionKernel {
public:
    virtual float compute(float x, float y) const = 0;
    virtual ~RegressionKernel() = default;
};

// Gaussian Radial Basis Function Kernel
class GaussianRBFKernel : public RegressionKernel {
private:
    float sigma;

public:
    GaussianRBFKernel(float bandwidth = 1.0f) : sigma(bandwidth) {}

    float compute(float x, float y) const override {
        float dist = std::abs(x - y);
        return std::exp(-0.5f * std::pow(dist / sigma, 2));
    }
};

// Adaptive Polynomial Regression with Machine Learning Guidance
class AdaptiveCurveFitter {
private:
    // Ensemble of curve fitters
    class CurveFitterModel {
    public:
        std::vector<float> coeffs;
        float confidence;
        std::unique_ptr<RegressionKernel> kernel;

        CurveFitterModel(int degree = 4) : 
            coeffs(degree + 1, 0.0f), 
            confidence(1.0f),
            kernel(std::make_unique<GaussianRBFKernel>()) {}

        float predict(float t) const {
            float prediction = 0.0f;
            for (size_t i = 0; i < coeffs.size(); ++i) {
                prediction += coeffs[i] * std::pow(t, i);
            }
            return prediction;
        }
    };

    // Machine learning inspired adaptive parameters
    struct LearningParameters {
        float baseLearnRate = 0.02f;
        float regularizationStrength = 0.01f;
        float momentumFactor = 0.9f;
        float explorationRate = 0.1f;
    };

    std::vector<CurveFitterModel> ensembleModels;
    LearningParameters learningParams;
    std::random_device rd;
    std::mt19937 generator;

    // Adaptive Momentum Learning Rate
    float computeAdaptiveLearningRate(int iter, float baseRate) {
        return baseRate / (1.0f + 0.1f * std::log(iter + 1));
    }

    // Stochastic Sample Selection
    std::vector<int> stochasticSampling(int totalCount, float sampleFraction = 0.5f) {
        std::vector<int> indices(totalCount);
        std::iota(indices.begin(), indices.end(), 0);
        
        int sampleSize = static_cast<int>(totalCount * sampleFraction);
        std::shuffle(indices.begin(), indices.end(), generator);
        indices.resize(sampleSize);
        
        return indices;
    }

    // L2 Regularization
    void applyRegularization(std::vector<float>& coeffs) {
        for (auto& coeff : coeffs) {
            coeff *= (1.0f - learningParams.regularizationStrength);
        }
    }

    // Kernel-weighted Error Computation
    float computeKernelWeightedError(
        const std::vector<DataPoint>& data, 
        const CurveFitterModel& model, 
        const RegressionKernel& kernel
    ) {
        float totalWeightedError = 0.0f;
        float totalWeight = 0.0f;

        for (size_t i = 0; i < data.size(); ++i) {
            float t = (data[i].timestamp - data[0].timestamp) / 
                      (data.back().timestamp - data[0].timestamp);
            
            float predicted = model.predict(t);
            float error = std::abs(data[i].value - predicted);
            
            // Compute kernel-based weight
            float weight = 0.0f;
            for (size_t j = 0; j < data.size(); ++j) {
                float neighborT = (data[j].timestamp - data[0].timestamp) / 
                                  (data.back().timestamp - data[0].timestamp);
                weight += kernel.compute(t, neighborT);
            }
            
            totalWeightedError += error * weight;
            totalWeight += weight;
        }

        return totalWeightedError / (totalWeight + 1e-8f);
    }

public:
    AdaptiveCurveFitter(int ensembleSize = 5, int polyDegree = 4) : 
        generator(rd()), 
        learningParams() 
    {
        // Initialize ensemble with diverse models
        for (int i = 0; i < ensembleSize; ++i) {
            ensembleModels.emplace_back(polyDegree);
            
            // Introduce initial diversity
            for (auto& coeff : ensembleModels.back().coeffs) {
                coeff = (std::rand() / static_cast<float>(RAND_MAX)) - 0.5f;
            }
        }
    }

    void advancedRefinement(
        std::vector<DataPoint>& data, 
        int maxIterations = 10
    ) {
        // Normalize timestamps
        float t0 = data.front().timestamp;
        float tMax = data.back().timestamp;
        float timeSpan = tMax - t0;

        for (int iter = 0; iter < maxIterations; ++iter) {
            float iterationTotalError = 0.0f;
            
            // Dynamic learning rate
            float currentLearnRate = computeAdaptiveLearningRate(
                iter, learningParams.baseLearnRate
            );

            // Stochastic sampling of data points
            auto sampledIndices = stochasticSampling(data.size());

            // Ensemble model update
            for (auto& model : ensembleModels) {
                float modelError = 0.0f;

                // Coefficient adjustment using sampled points
                for (int idx : sampledIndices) {
                    float t = (data[idx].timestamp - t0) / timeSpan;
                    float predicted = model.predict(t);
                    float error = data[idx].value - predicted;

                    // Adaptive coefficient update with kernel weighting
                    for (size_t j = 0; j < model.coeffs.size(); ++j) {
                        float kernelWeight = model.kernel->compute(t, 0.5f);
                        model.coeffs[j] += currentLearnRate * 
                                           error * 
                                           std::pow(t, j) * 
                                           kernelWeight;
                    }

                    modelError += std::abs(error);
                }

                // Regularization
                applyRegularization(model.coeffs);

                // Update model confidence
                model.confidence = 1.0f / (1.0f + modelError);
                iterationTotalError += modelError;
            }

            // Early stopping condition
            if (iterationTotalError < 1e-6f) break;
        }
    }

    // Ensemble prediction with weighted averaging
    float predict(float timestamp, float t0, float timeSpan) {
        float t = (timestamp - t0) / timeSpan;
        float prediction = 0.0f;
        float totalConfidence = 0.0f;

        for (const auto& model : ensembleModels) {
            prediction += model.predict(t) * model.confidence;
            totalConfidence += model.confidence;
        }

        return prediction / (totalConfidence + 1e-8f);
    }
};
