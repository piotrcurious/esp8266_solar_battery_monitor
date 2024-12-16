// ... (Other includes and structs)

VectorXd fitPolynomialLeastSquares(const std::vector<std::pair<double, double>>& data, int order); // Function declaration from above

Polynomial monteCarloFit(const PolynomialBuffer& buffer, unsigned long totalDuration) { //Renamed and now uses least squares
    if(!buffer.isFull()){
        Serial.println("Buffer not full, cannot perform fit.");
        return {VectorXd::Zero(0), 0};
    }

    std::vector<std::pair<double, double>> dataPoints;
    for (unsigned long t = 0; t <= totalDuration; t+= totalDuration/1000) {
        double trueValue = 0;
        for(const auto& poly : buffer.polynomials){
            if(t >= poly.startTime && t < poly.startTime + totalDuration/buffer.size) {
                trueValue = evaluatePolynomial(poly.coefficients, t - poly.startTime);
                break;
            }
        }
        dataPoints.push_back({(double)t, trueValue});
    }

    Polynomial bestPolynomial;
    bestPolynomial.coefficients = fitPolynomialLeastSquares(dataPoints, 8); // Fit 8th order
    bestPolynomial.startTime = buffer.polynomials[0].startTime;
    return bestPolynomial;
}

//... (rest of the code, loop, setup)
