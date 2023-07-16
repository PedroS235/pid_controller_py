from pid_controller import PIDController


def main():
    pid = PIDController(1, 0.1, 0.5, -1, 1)
    pid  # Just to ingore the linting warning
    print("This is a PID Controller Python Package")


if __name__ == "__main__":
    main()
