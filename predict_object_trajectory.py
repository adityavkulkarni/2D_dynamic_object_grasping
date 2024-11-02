import numpy as np

# Hybrid model: Polynomial + Trigonometric Regression
class HybridCurveFitter:
    def __init__(self, degree, omega, lam=1.0, delta=1e5):
        """
        Initialize a hybrid curve fitter that combines polynomial and trigonometric regression.

        Parameters:
            degree (int): Degree of the polynomial.
            omega (float): Angular frequency of the sine and cosine terms.
            lam (float): Forgetting factor (controls weight for recent data).
            delta (float): Initial value for P matrix (should be large).
        """
        self.omega = omega
        self.lam = lam

        # Total number of coefficients: (degree + 1) for polynomial, 2 for trigonometric
        self.num_coeffs = degree + 1 + 2

        # Initialize parameters for [polynomial terms + trig terms]
        self.theta = np.zeros(self.num_coeffs)

        # Covariance matrix for RLS
        self.P = np.eye(self.num_coeffs) * delta

    def update(self, t, y):
        """
        Update the hybrid model based on new observation (t, y).

        Parameters:
            t (float): Time point.
            y (float): Observed value at time t.
        """
        # Create the regression vector for polynomial and trig terms
        poly_terms = np.array([t ** i for i in range(self.num_coeffs - 2)])  # Polynomial terms
        trig_terms = np.array([np.cos(self.omega * t), np.sin(self.omega * t)])  # Trigonometric terms
        phi = np.concatenate([poly_terms, trig_terms])  # Combine both

        # Compute gain factor
        P_phi = np.dot(self.P, phi)
        K = P_phi / (self.lam + np.dot(phi, P_phi))

        # Update the coefficients
        self.theta = self.theta + K * (y - np.dot(self.theta, phi))

        # Update the covariance matrix
        self.P = (self.P - np.outer(K, P_phi)) / self.lam

    def predict(self, t):
        """
        Predict the value at time t using the fitted hybrid model.

        Parameters:
            t (float): Time point to predict.

        Returns:
            float: Predicted value at time t.
        """
        poly_terms = np.array([t ** i for i in range(self.num_coeffs - 2)])  # Polynomial terms
        trig_terms = np.array([np.cos(self.omega * t), np.sin(self.omega * t)])  # Trigonometric terms
        phi = np.concatenate([poly_terms, trig_terms])

        return np.dot(self.theta, phi)