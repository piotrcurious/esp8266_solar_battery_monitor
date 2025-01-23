#include <Arduino.h>
#include <vector>
#include <complex>
#include <algorithm>
#include <numeric>
#include <limits>

template<typename Field = double>
class RobustPolynomialCodec {
public:
    // Polynomial representation
    using Polynomial = std::vector<Field>;
    using PolynomialSet = std::vector<Polynomial>;

    // Compression result structure (enhanced from previous implementation)
    struct CompressionDescriptor {
        enum class ReductionStatus {
            FULLY_REDUCIBLE,
            PARTIALLY_REDUCIBLE,
            IRREDUCIBLE
        };

        Polynomial originalPolynomial;
        Polynomial commonFactor;
        PolynomialSet reducedComponents;
        ReductionStatus reductionStatus;
        
        // Additional metadata for reconstruction
        std::vector<int> componentDegrees;
        Field scalingFactor = 1.0;
        
        // Metrics
        double compressionRatio = 0.0;
        double reconstructionError = std::numeric_limits<double>::max();
    };

    // Compression Strategy (similar to previous implementation)
    class PolynomialCompressionStrategy {
    public:
        static CompressionDescriptor compress(const Polynomial& poly) {
            CompressionDescriptor result;
            result.originalPolynomial = poly;

            // Existing compression logic from previous implementation
            // ... (keep the previous compression method)

            // Enhanced metadata tracking
            result.componentDegrees.clear();
            for (const auto& component : result.reducedComponents) {
                result.componentDegrees.push_back(component.size() - 1);
            }

            return result;
        }
    };

    // Advanced Decompression Strategy
    class PolynomialDecompressionStrategy {
    public:
        // Main decompression method
        static Polynomial decompress(const CompressionDescriptor& descriptor) {
            switch (descriptor.reductionStatus) {
                case CompressionDescriptor::ReductionStatus::IRREDUCIBLE:
                    return decompressIrreducible(descriptor);
                
                case CompressionDescriptor::ReductionStatus::PARTIALLY_REDUCIBLE:
                    return decompressPartiallyReducible(descriptor);
                
                case CompressionDescriptor::ReductionStatus::FULLY_REDUCIBLE:
                    return decompressFullyReducible(descriptor);
                
                default:
                    // Fallback to original polynomial
                    return descriptor.originalPolynomial;
            }
        }

    private:
        // Decompress irreducible polynomial
        static Polynomial decompressIrreducible(const CompressionDescriptor& descriptor) {
            // For irreducible polynomials, return the original
            return descriptor.originalPolynomial;
        }

        // Decompress partially reducible polynomial
        static Polynomial decompressPartiallyReducible(const CompressionDescriptor& descriptor) {
            // Reconstruct by multiplying reduced components
            return multiplyPolynomials(descriptor.reducedComponents);
        }

        // Decompress fully reducible polynomial
        static Polynomial decompressFullyReducible(const CompressionDescriptor& descriptor) {
            // Similar to partially reducible, but with potential additional processing
            Polynomial reconstructed = multiplyPolynomials(descriptor.reducedComponents);
            
            // Apply scaling factor if needed
            if (std::abs(descriptor.scalingFactor - 1.0) > 1e-10) {
                std::transform(reconstructed.begin(), reconstructed.end(), reconstructed.begin(),
                    [&](Field coeff) { return coeff * descriptor.scalingFactor; }
                );
            }

            return reconstructed;
        }

        // Utility: Multiply multiple polynomials
        static Polynomial multiplyPolynomials(const PolynomialSet& polynomials) {
            if (polynomials.empty()) return {};
            
            Polynomial result = {1.0};
            for (const auto& poly : polynomials) {
                result = multiplyTwoPolynomials(result, poly);
            }
            
            return result;
        }

        // Utility: Multiply two polynomials
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

    // Comprehensive Codec Interface
    class PolynomialCodec {
    public:
        // Compress a polynomial
        static CompressionDescriptor compress(const Polynomial& poly) {
            return PolynomialCompressionStrategy::compress(poly);
        }

        // Decompress a compression descriptor
        static Polynomial decompress(const CompressionDescriptor& descriptor) {
            return PolynomialDecompressionStrategy::decompress(descriptor);
        }

        // Verify reconstruction quality
        static bool verifyReconstruction(
            const Polynomial& original, 
            const Polynomial& reconstructed,
            Field toleranceThreshold = 1e-6
        ) {
            if (original.size() != reconstructed.size()) return false;
            
            for (size_t i = 0; i < original.size(); ++i) {
                if (std::abs(original[i] - reconstructed[i]) > toleranceThreshold) {
                    return false;
                }
            }
            
            return true;
        }
    };

public:
    // Simplified interface methods
    static CompressionDescriptor Compress(const Polynomial& poly) {
        return PolynomialCodec::compress(poly);
    }

    static Polynomial Decompress(const CompressionDescriptor& descriptor) {
        return PolynomialCodec::decompress(descriptor);
    }
};

void setup() {
    Serial.begin(115200);

    // Type definition
    using PolynomialCodec = RobustPolynomialCodec<double>;
    
    // Sample polynomials
    std::vector<std::vector<double>> testPolynomials = {
        {1.0, -6.0, 11.0, -6.0},  // Reducible: (x-1)(x-2)(x-3)
        {1.0, 0.0, 1.0},           // Irreducible over reals
        {1.0, 2.0, 1.0}            // Partially reducible
    };

    // Compression and Decompression Test
    for (const auto& poly : testPolynomials) {
        Serial.println("Polynomial Compression-Decompression Test:");
        
        // Print original polynomial
        Serial.print("Original Polynomial: ");
        for (const auto& coeff : poly) {
            Serial.print(coeff);
            Serial.print(" ");
        }
        Serial.println();

        // Compress
        auto compressionDescriptor = PolynomialCodec::Compress(poly);

        // Decompress
        auto reconstructedPoly = PolynomialCodec::Decompress(compressionDescriptor);

        // Verify reconstruction
        bool isReconstructed = PolynomialCodec::PolynomialCodec::verifyReconstruction(poly, reconstructedPoly);

        // Print results
        Serial.print("Reconstruction Status: ");
        Serial.println(isReconstructed ? "Success" : "Failed");

        Serial.print("Reconstructed Polynomial: ");
        for (const auto& coeff : reconstructedPoly) {
            Serial.print(coeff);
            Serial.print(" ");
        }
        Serial.println("\n---");
    }
}

void loop() {
    // Main application logic
}
