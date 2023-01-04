package frc.robot

/**
 * The RobotState class is a singleton that keeps track of the current state of the robot.
 */
enum class RobotState {
    DISABLED,
    AUTONOMOUS,
    TELEOP,
    TEST,
}