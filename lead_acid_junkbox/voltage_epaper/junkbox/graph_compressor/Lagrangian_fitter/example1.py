import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import Ridge

class LagrangianPolynomialFitter:
    def __init__(self, degree=3, lambda_reg=1.0):
        """
        Initialize polynomial fitter with Lagrangian regularization
        
        Args:
            degree (int): Degree of polynomial to fit
            lambda_reg (float): Regularization strength (Lagrange multiplier)
        """
        self.degree = degree
        self.lambda_reg = lambda_reg
        self.model = None
        self.poly_features = None
    
    def fit(self, X, y):
        """
        Fit polynomial using Ridge regression (equivalent to Lagrangian dual formulation)
        
        Args:
            X (array): Input timestamps
            y (array): Corresponding data points
        """
        # Transform input to polynomial features
        self.poly_features = PolynomialFeatures(degree=self.degree)
        X_poly = self.poly_features.fit_transform(X.reshape(-1, 1))
        
        # Ridge regression (solves Lagrangian dual problem)
        self.model = Ridge(alpha=self.lambda_reg, fit_intercept=False)
        self.model.fit(X_poly, y)
        
        return self
    
    def predict(self, X):
        """
        Predict values using fitted polynomial
        
        Args:
            X (array): Input timestamps for prediction
        
        Returns:
            array: Predicted values
        """
        X_poly = self.poly_features.transform(X.reshape(-1, 1))
        return self.model.predict(X_poly)
    
    def plot_results(self, X, y):
        """
        Visualize the polynomial fit
        
        Args:
            X (array): Original input timestamps
            y (array): Original data points
        """
        plt.figure(figsize=(10, 6))
        plt.scatter(X, y, color='blue', label='Original Data')
        
        # Generate smooth prediction line
        X_pred = np.linspace(X.min(), X.max(), 200)
        y_pred = self.predict(X_pred)
        
        plt.plot(X_pred, y_pred, color='red', label=f'Polynomial Fit (Degree {self.degree})')
        plt.title('Lagrangian Polynomial Fitting')
        plt.xlabel('Timestamps')
        plt.ylabel('Data Points')
        plt.legend()
        plt.show()

# Example usage
def generate_noisy_data(n_samples=100):
    """Generate synthetic data with noise"""
    X = np.linspace(0, 10, n_samples)
    true_coeffs = [1, 0.5, -0.1, 0.01]
    y = true_coeffs[0] + true_coeffs[1]*X + true_coeffs[2]*X**2 + true_coeffs[3]*X**3
    y += np.random.normal(0, 1, n_samples)  # Add noise
    return X, y

# Demonstration
X, y = generate_noisy_data()

# Try different regularization strengths
for lambda_reg in [0.1, 1.0, 10.0]:
    fitter = LagrangianPolynomialFitter(degree=3, lambda_reg=lambda_reg)
    fitter.fit(X, y)
    fitter.plot_results(X, y)
