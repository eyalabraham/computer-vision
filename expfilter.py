###########################################################
#
# expfilter.py
#
#   Exponential filter class.
#   Provide exponential smoothing to noisy input.
#   For example, this filter can be used before an PID stage.
#   Resource: https://en.wikipedia.org/wiki/Exponential_smoothing
#
#   February 11, 2018
#

class ExpFilter(object):
    """Calculate an exponential filter algorithm on noisy input data."""

    def __init__(self, alpha = 0.1, init = 0.0):
        """
        Initialize the t0 value of the filter, and the filter's alpha coefficient.
        The constructor will throw an exception if alpha is
        less than 0.0 or greater-than-or-equal to 1.0.
        """

        if alpha < 0.0 or alpha >= 1.0:
            raise ValueError('Must be: 0.0 < alpha < 1.0')

        self.filtered_err_prev = init
        self.alpha = alpha
    
    def get_alpha(self):
        """Retrieve the filter's alpha value"""

        return self.alpha
    
    def set_alpha(self, alpha = 0.1):
        """
        Change filter's alpha to a new value or reset it to default of 0.1.
        The function will throw an exception if alpha is
        less than 0.0 or greater-than-or-equal to 1.0.
        """

        if alpha < 0.0 or alpha >= 1.0:
            raise ValueError('Must be: 0.0 < alpha < 1.0')

        self.alpha = alpha
   
    def reset(self, alpha = 0.1, init = 0.0):
        """
        Reset the internal filter state before reusing.
        This is useful when reusing a filter without reconstituting a new filter object.
        """
        
        if alpha < 0.0 or alpha >= 1.0:
            raise ValueError('Must be: 0.0 < alpha < 1.0')

        self.filtered_err_prev = init
        self.alpha = alpha
    
    def filter(self, data):
        """Calculate and return a filtered float value of the input data."""

        self.filtered_err_prev = (self.alpha * data) + ((1 - self.alpha) * self.filtered_err_prev)
        return self.filtered_err_prev

