# Created by Philip Smith                           %                                                                               
#       Email: psmail147@gmail.com                  %                                                    
#       Github: https://github.com/psmail147        %                         
# --------------------------------------------------%

from collections import deque
import math

class PID_Controller:
    """Standard PID controller implementation."""
    
    def __init__(self):
        # Controller gains
        self.proportional_gain_constant = 0.2 
        self.integral_gain_constant = 0.01
        self.derivative_gain_constant = 1.0
        
        # Integral term accumulator
        self.accumulated_error = 0

    def proportional_response(self, error):
        """Compute proportional response based on current error."""
        return self.proportional_gain_constant * error

    def integral_response(self):
        """Compute integral response based on accumulated past errors."""
        return self.integral_gain_constant * self.accumulated_error

    def derivative_response(self, delta_error, time_delta):
        """Compute derivative response based on change in error over time."""
        return self.derivative_gain_constant * (delta_error / time_delta)



# The below class was created to test stochastic otpimization techniques only as it clear that fine tuning
# a classic PID controller is overkill.  

class PID_Controller_Fract:
    """Fractional-order PID controller (FOPID) using discrete approximations."""

    def __init__(self,
                 Kp=0.2,
                 Ki=0.01,
                 Kd=1.0,
                 lambda_order=0.2,
                 mu_order=0.5,
                 max_history=50):
        # Controller gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # Fractional orders for integration (λ) and differentiation (μ)
        self.lambda_order = lambda_order
        self.mu_order = mu_order
        
        # Error and time histories, limited to max_history entries
        self.error_history = deque(maxlen=max_history)
        self.time_history = deque(maxlen=max_history)

    def update(self, error, current_time):
        """
        Update the controller with the latest error and time.
        Compute the control output based on fractional PID logic.
        """
        self.error_history.append(error)
        self.time_history.append(current_time)

        # Compute elapsed time between last two samples
        if len(self.time_history) >= 2:
            dt = self.time_history[-1] - self.time_history[-2]
        else:
            dt = 1e-3  # Default small dt to prevent division by zero

        # Proportional term (standard)
        P_term = self.Kp * error

        # Fractional integral term using Grünwald–Letnikov approximation
        I_term = 0
        for k, e_k in enumerate(reversed(self.error_history)):
            coeff = (dt ** self.lambda_order) / math.gamma(self.lambda_order + 1)
            I_term += coeff * e_k
        I_term *= self.Ki

        # Fractional derivative term using finite difference method
        D_term = 0
        for k in range(1, len(self.error_history)):
            delta_e = self.error_history[-k] - self.error_history[-k - 1]
            weight = (-1) ** k * self._frac_binomial_coeff(self.mu_order, k)
            D_term += weight * delta_e
        D_term *= self.Kd / dt

        # Total control signal
        control_output = P_term + I_term + D_term
        return control_output

    def _frac_binomial_coeff(self, alpha, k):
        """
        Compute generalized binomial coefficient for fractional exponents.
        Used for fractional derivative approximation.
        """
        numerator = 1
        for i in range(k):
            numerator *= (alpha - i)
        denominator = math.factorial(k)
        return numerator / denominator
