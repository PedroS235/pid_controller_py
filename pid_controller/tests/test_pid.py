import pytest
from pid_controller.pid import PID


@pytest.fixture
def pid():
    return PID(pid_gains=(1, 1, 1))


def test_pid_gains(pid):
    pid.set_pid_gains((2, 2, 2))
    assert pid.get_pid_gains() == (2, 2, 2)


def test_pid_gains_with_wrong_number_of_values(pid):
    with pytest.raises(AssertionError):
        pid.set_pid_gains((1, 2, 3, 4))


def test_output_limits(pid):
    pid.set_output_limits((-2, 2))
    assert pid.get_output_limits() == (-2, 2)


def test_output_limits_with_wrong_number_of_values(pid):
    with pytest.raises(AssertionError):
        pid.set_output_limits((1, 2, 3))


def test_setpoint(pid):
    pid.set_setpoint(10)
    assert pid.get_setpoint() == 10


def test_zero_error(pid):
    pid.set_setpoint(0)
    assert pid.compute(0) == 0


def test_proportional_gain(pid):
    pid.set_pid_gains((2, 0, 0))
    pid.set_setpoint(10)
    pid.set_output_limits((float("-inf"), float("inf")))
    assert pid.compute(0) == 20


def test_integral_gain(pid):
    pid.set_pid_gains((0, 2, 0))
    pid.set_setpoint(10)
    pid.set_output_limits((float("-inf"), float("inf")))
    assert pid.compute(0) == 20


def test_derivative_gain(pid):
    pid.set_pid_gains((0, 0, 2))
    pid.set_setpoint(10)
    pid.set_output_limits((float("-inf"), float("inf")))
    assert pid.compute(0) == 20


def test_p_i_gains(pid):
    pid.set_pid_gains((2, 2, 0))
    pid.set_setpoint(10)
    pid.set_output_limits((float("-inf"), float("inf")))
    assert pid.compute(5) == 20


def test_p_d_i_gains(pid):
    pid.set_pid_gains((2, 2, 2))
    pid.set_setpoint(10)
    pid.set_output_limits((float("-inf"), float("inf")))
    assert pid.compute(5) == 30


def test_p_d_i_gains_with_output_limits(pid):
    pid.set_pid_gains((2, 2, 2))
    pid.set_setpoint(10)
    pid.set_output_limits((-10, 10))
    assert pid.compute(5) == 10
