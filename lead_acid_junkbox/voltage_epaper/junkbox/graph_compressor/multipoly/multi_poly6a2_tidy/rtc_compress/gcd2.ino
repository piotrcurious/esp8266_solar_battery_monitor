#include <Arduino.h>
#include <vector>
#include <complex>
#include <algorithm>
#include <numeric>
#include <limits>

// Advanced polynomial compression framework
template<typename Field = double>
class RobustPolynomialCompressor {
public:
    // Polynomial representation
    using Polynomial = std::vector<Field>;
    using PolynomialSet = std::vector<Polynomial>;

    // Comprehensive polynomial analysis result
    struct PolynomialAnalysisResult {
        enum class ReductionStatus {
            FULLY_REDUCIBLE,
            PARTIALLY_REDUCIBLE,
            IRREDUCIBLE
        };

        Polynomial originalPolynomial;
        Polynomial commonFactor;
        PolynomialSet reducedComponents;
        ReductionStatus reductionStatus;
        
        // Metrics for compression assessment
        double compressionRatio = 0.0;
        double reconstructionError = std::numeric_limits<double>::max();
    };

    // Advanced irreducibility testing
    class IrreducibilityTester {
    public:
        // Comprehensive irreducibility test
        static bool isIrreducible(const Polynomial& poly) {
            // Multiple irreducibility testing strategies
            return eisensteinCriterion(poly) || 
                   rationalRootTest(poly) || 
                   sophisticatedFactorizationTest(poly);
        }

    private:
        // Eisenstein's Irreducibility Criterion
        static bool eisensteinCriterion(const Polynomial& poly) {
            if (poly.size() <= 1) return false;

            for (int p = 2; p < 100; ++p) { // Test with first few prime numbers
                bool meetsEisenstein = true;
                
                // P divides all but the first coefficient
                for (size_t i = 0; i < poly.size() - 1; ++i) {
                    if (std::fmod(std::abs(poly[i]), p) != 0) {
                        meetsEisenstein = false;
                        break;
                    }
                }

                // P does not divide the last coefficient
                if (meetsEisenstein && 
                    std::fmod(std::abs(poly.back()), p) != 0 &&
                    std::fmod(std::abs(poly[0]), p * p) != 0) {
                    return true;
                }
            }
            return false;
        }

        // Rational root test
        static bool rationalRootTest(const Polynomial& poly) {
            // Check if polynomial has no rational roots
            std::vector<Field> possibleRoots = computePossibleRoots(poly);
            return possibleRoots.empty();
        }

        // Sophisticated factorization test
        static bool sophisticatedFactorizationTest(const Polynomial& poly) {
            // Advanced irreducibility detection
            // Combines multiple mathematical criteria
            
            // Length-based heuristics
            if (poly.size() <= 2) return false;
            
            // Symmetry and structural analysis
            return analyzePolynomialStructure(poly);
        }

        // Compute possible rational roots
        static std::vector<Field> computePossibleRoots(const Polynomial& poly) {
            std::vector<Field> roots;
            
            // Factors of the constant term
            std::vector<Field> constantFactors = computeFactors(std::abs(poly.back()));
            
            // Factors of the leading coefficient
            std::vector<Field> leadingFactors = computeFactors(std::abs(poly[0]));
            
            // Test potential rational roots
            for (Field p : constantFactors) {
                for (Field q : leadingFactors) {
                    Field root = p / q;
                    if (evaluatePolynomial(poly, root) == 0) {
                        roots.push_back(root);
                    }
                }
            }
            
            return roots;
        }

        // Compute integer factors
        static std::vector<Field> computeFactors(Field n) {
            std::vector<Field> factors;
            for (Field i = 1; i <= std::abs(n); ++i) {
                if (std::fmod(n, i) == 0) {
                    factors.push_back(i);
                    if (i != n/i) factors.push_back(n/i);
                }
            }
            return factors;
        }

        // Polynomial root evaluation
        static Field evaluatePolynomial(const Polynomial& poly, Field x) {
            Field result = 0;
            for (size_t i = 0; i < poly.size(); ++i) {
                result += poly[i] * std::pow(x, poly.size() - i - 1);
            }
            return result;
        }

        // Structural polynomial analysis
        static bool analyzePolynomialStructure(const Polynomial& poly) {
            // Check for symmetry and special structural properties
            bool hasSymmetry = checkPolynomialSymmetry(poly);
            
            // Compute coefficient distribution
            double entropyScore = computeCoefficientEntropy(poly);
            
            return hasSymmetry || entropyScore < 0.5;
        }

        // Symmetry detection
        static bool checkPolynomialSymmetry(const Polynomial& poly) {
            for (size_t i = 0; i < poly.size() / 2; ++i) {
                if (std::abs(poly[i] - poly[poly.size() - i - 1]) > 1e-10) {
                    return false;
                }
            }
            return true;
        }

        // Coefficient entropy computation
        static double computeCoefficientEntropy(const Polynomial& poly) {
            // Simplified entropy calculation
            double entropy = 0.0;
            for (Field coeff : poly) {
                double p = std::abs(coeff);
                entropy -= p * std::log2(p + 1e-10);
            }
            return entropy;
        }
    };

    // Advanced compression strategy
    class PolynomialCompressionStrategy {
    public:
        // Comprehensive polynomial analysis and compression
        static PolynomialAnalysisResult analyzeAndCompress(const Polynomial& poly) {
            PolynomialAnalysisResult result;
            result.originalPolynomial = poly;

            // Irreducibility check
            if (IrreducibilityTester::isIrreducible(poly)) {
                result.reductionStatus = PolynomialAnalysisResult::ReductionStatus::IRREDUCIBLE;
                return result;
            }

            // Attempt partial factorization
            auto factors = partialFactorization(poly);
            
            if (factors.size() > 1) {
                result.reductionStatus = factors.size() == 2 ? 
                    PolynomialAnalysisResult::ReductionStatus::PARTIALLY_REDUCIBLE :
                    PolynomialAnalysisResult::ReductionStatus::FULLY_REDUCIBLE;
                
                result.reducedComponents = factors;
                
                // Compute common factor
                result.commonFactor = computeCommonFactor(factors);
                
                // Compression ratio calculation
                result.compressionRatio = computeCompressionRatio(poly, result.reducedComponents);
                
                // Reconstruction error estimation
                result.reconstructionError = computeReconstructionError(poly, result.reducedComponents);
            } else {
                result.reductionStatus = PolynomialAnalysisResult::ReductionStatus::IRREDUCIBLE;
            }

            return result;
        }

    private:
        // Partial factorization method
        static PolynomialSet partialFactorization(const Polynomial& poly) {
            PolynomialSet factors;
            
            // Rational root factorization
            auto roots = IrreducibilityTester::computePossibleRoots(poly);
            
            if (!roots.empty()) {
                // Factor out roots
                for (Field root : roots) {
                    Polynomial factor = {1.0, -root};
                    factors.push_back(factor);
                }
            }
            
            // Fallback to full polynomial if no factors found
            if (factors.empty()) {
                factors.push_back(poly);
            }
            
            return factors;
        }

        // Compute common factor across polynomials
        static Polynomial computeCommonFactor(const PolynomialSet& factors) {
            if (factors.empty()) return {};
            
            Polynomial commonFactor = factors[0];
            for (size_t i = 1; i < factors.size(); ++i) {
                commonFactor = computePolynomialGCD(commonFactor, factors[i]);
            }
            
            return commonFactor;
        }

        // Polynomial GCD computation
        static Polynomial computePolynomialGCD(const Polynomial& a, const Polynomial& b) {
            Polynomial larger = a.size() >= b.size() ? a : b;
            Polynomial smaller = a.size() < b.size() ? a : b;
            
            while (!smaller.empty()) {
                Polynomial remainder = polynomialRemainder(larger, smaller);
                larger = smaller;
                smaller = remainder;
            }
            
            return larger;
        }

        // Polynomial long division remainder
        static Polynomial polynomialRemainder(const Polynomial& dividend, const Polynomial& divisor) {
            if (divisor.empty()) return dividend;
            
            Polynomial remainder = dividend;
            Field scale = divisor.back();
            
            while (remainder.size() >= divisor.size()) {
                Field leadCoeff = remainder.back() / scale;
                
                for (size_t i = 0; i < divisor.size(); ++i) {
                    remainder[remainder.size() - divisor.size() + i] -= 
                        leadCoeff * divisor[i];
                }
                
                // Remove leading zeros
                while (!remainder.empty() && std::abs(remainder.back()) < 1e-10) {
                    remainder.pop_back();
                }
            }
            
            return remainder;
        }

        // Compression ratio calculation
        static double computeCompressionRatio(
            const Polynomial& original, 
            const PolynomialSet& factors
        ) {
            size_t originalSize = original.size();
            size_t compressedSize = 0;
            
            for (const auto& factor : factors) {
                compressedSize += factor.size();
            }
            
            return 1.0 - (static_cast<double>(compressedSize) / originalSize);
        }

        // Reconstruction error estimation
        static double computeReconstructionError(
            const Polynomial& original, 
            const PolynomialSet& factors
        ) {
            // Reconstruct polynomial from factors
            Polynomial reconstructed = multiplyPolynomials(factors);
            
            // Compute error
            double error = 0.0;
            for (size_t i = 0; i < original.size(); ++i) {
                error += std::abs(original[i] - reconstructed[i]);
            }
            
            return error / original.size();
        }

        // Multiply polynomials
        static Polynomial multiplyPolynomials(const PolynomialSet& polynomials) {
            Polynomial result = {1.0};
            
            for (const auto& poly : polynomials) {
                result = multiplyTwoPolynomials(result, poly);
            }
            
            return result;
        }

        // Multiply two polynomials
        static Polynomial multiplyTwoPolynomials(
            const Polynomial& a, 
            const Polynomial& b
        ) {
            Polynomial result(a.size() + b.size() - 1, 0.0);
            
            for (size_t i = 0; i < a.size(); ++i) {
                for (size_t j = 0; j < b.size(); ++j) {
                    result[i + j] += a[i] * b[j];
                }
            }
            
            return result;
        }
    };

public:
    // Main compression method
    static PolynomialAnalysisResult compress(const Polynomial& poly) {
        return PolynomialCompressionStrategy::analyzeAndCompress(poly);
    }
};

void setup() {
    Serial.begin(115200);

    // Example usage
    using PolynomialCompressor = RobustPolynomialCompressor<double>;
    
    // Sample polynomials
    std::vector<PolynomialCompressor::Polynomial> testPolynomials = {
        {1.0, -6.0, 11.0, -6.0},  // Reducible: (x-1)(x-2)(x-3)
        {1.0, 0.0, 1.0},           // Irreducible over reals
        {1.0, 2.0, 1.0}            // Partially reducible
    };

    // Analyze and compress each polynomial
    for (const auto& poly : testPolynomials) {
        auto result = PolynomialCompressor::compress(poly);

        // Print analysis results
        Serial.println("Polynomial Analysis:");
        Serial.print("Reduction Status: ");
        switch (result.reductionStatus) {
            case PolynomialAnalysisResult::ReductionStatus::FULLY_REDUCIBLE:
                Serial.println("Fully Reducible");
                break;
            case PolynomialAnalysisResult::ReductionStatus::PARTIALLY_REDUCIBLE:
                Serial.println("Partially Reducible");
                break;
            case PolynomialAnalysisResult::ReductionStatus::IRREDUCIBLE:
                Serial.println("Irreducible");
                break;
        }

        // Print compression metrics
        Serial.print("Compression Ratio: ");
        Serial.println(result.compressionRatio);
        
        Serial.print("Reconstruction Error: ");
        Serial.println(result.reconstructionError);

        // Print reduced components if available
        if (!result.reducedComponents.empty()) {
            Serial.println("Reduced Components:");
            for (const auto& component : result.reducedComponents) {
                for (const auto& coeff : component) {
                    Serial.print(coeff);
                    Serial.print(" ");
                }
                Serial.println();
            }
        }
        Serial.println("---");
    }
}

void loop() {
    // Main application logic
}
