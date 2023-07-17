class PID:
    _pid_gains: tuple
    _output_limits: tuple
    _error_sum: float
    _prev_error: float
    _setpoint: float

    def __init__(
        self,
        pid_gains: tuple,
        output_limits = (0, 255),
    ) -> None:
        """Constructor for the PID class

        Args:
            pid_gains (tuple): The PID gains (kp, ki, kd)
            output_limits (tuple, optional): The output limits (min, max). Defaults to (0, 255).

        Raises:
            AssertionError: If the pid_gains parameter is not of length 3
            AssertionError: If the pid_gains parameter contains a negative value
            AssertionError: If the output_limits parameter is not of length 2
            AssertionError: If the output_limits parameter is not of the form (min, max)

        Returns:
            None
        """
        self._assert_pid_gains(pid_gains)
        self._assert_output_limits(output_limits)

        self._pid_gains = pid_gains
        self._output_limits = output_limits

    def set_output_limits(self, output_limits: tuple) -> None:
        """Sets the output limits

        Args:
            output_limits (tuple): The output limits (min, max)

        Raises:
            AssertionError: If the output_limits parameter is not of length 2
            AssertionError: If the output_limits parameter is not of the form (min, max)

        Returns:
            None
        """
        self._assert_output_limits(output_limits)
        self._output_limits = output_limits

    def set_pid_gains(self, pid_gains: tuple) -> None:
        """Sets the PID gains

        Args:
            pid_gains (tuple): The PID gains (kp, ki, kd)

        Raises:
            AssertionError: If the pid_gains parameter is not of length 3
            AssertionError: If the pid_gains parameter contains a negative value

        Returns:
            None
        """
        self._assert_pid_gains(pid_gains)
        self._pid_gains = pid_gains

    def set_setpoint(self, setpoint: float) -> None:
        """Sets the setpoint

        Args:
            setpoint (float): The setpoint

        Returns:
            None
        """
        self.reset()
        self._setpoint = setpoint

    def get_pid_gains(self) -> tuple:
        """Returns the PID gains

        Returns:
            tuple: The PID gains (kp, ki, kd)
        """
        return self._pid_gains

    def get_output_limits(self) -> tuple:
        """Returns the output limits

        Returns:
            tuple: The output limits (min, max)
        """
        return self._output_limits

    def get_setpoint(self) -> float:
        """Returns the setpoint

        Returns:
            float: The setpoint
        """
        return self._setpoint

    def reset(self) -> None:
        """Resets the PID controller

        Returns:
            None
        """
        self._prev_error = 0
        self._error_sum = 0

    def compute(self, measured_value: float) -> float:
        """Computes the PID correction based on the measured value

        Args:
            measured_value (float): The measured value

        Returns:
            float: The PID correction
        """
        kp, ki, kd = self._pid_gains
        min_output, max_output = self._output_limits
        error = self._setpoint - measured_value
        self._error_sum += error

        self._error_sum = self._bound_value(
            self._error_sum, min_output, max_output
        )

        p_term = kp * error
        i_term = ki * self._error_sum
        d_term = kd * (error - self._prev_error)

        self._prev_error = error

        correction = p_term + i_term + d_term
        return self._bound_value(correction, min_output, max_output)

    def _bound_value(
        self, value: float, min_value: float, max_value: float
    ) -> float:
        """Bounds the value between the min and max values

        Args:
            value (float): The value to bound
            min_value (float): The minimum value
            max_value (float): The maximum value

        Returns:
            float: The bounded value
        """
        if value > max_value:
            return max_value
        elif value < min_value:
            return min_value
        else:
            return value

    def _assert_pid_gains(self, pid_gains: tuple) -> None:
        """Asserts that the pid_gains parameter is valid

        Args:
            pid_gains (tuple): The PID gains (kp, ki, kd)

        Raises:
            AssertionError: If the pid_gains parameter is not of length 3
            AssertionError: If the pid_gains parameter contains a negative value

        Returns:
            None
        """
        assert (
            len(pid_gains) == 3
        ), "The pid_gains parameter should only contain 3 values: kp, ki, kd"

        assert (
            pid_gains[0] >= 0 and pid_gains[1] >= 0 and pid_gains[2] >= 0
        ), "All pid gains must be a positive float"

    def _assert_output_limits(self, output_limits: tuple) -> None:
        """Asserts that the output_limits parameter is valid

        Args:
            output_limits (tuple): The output limits (min, max)


        Raises:
            AssertionError: If the output_limits parameter is not of length 2
            AssertionError: If the output_limits parameter is not of the form (min, max)

        Returns:
            None
        """
        assert (
            len(output_limits) == 2
        ), "The output_limits parameter should only have a max & min value"

        assert (
            output_limits[0] < output_limits[1]
        ), "Please make sure the output_limits is of the form: (min, max)"
