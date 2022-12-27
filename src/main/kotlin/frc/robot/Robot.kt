package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import frc.robot.commands.DriveCommand
import frc.robot.subsystems.Drivetrain

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {
    /**
     * xbox controller
     */
    val xbox = XboxController(0)

    /**
     * drive subsystem
     */
    val drivetrain = Drivetrain()

    private var robotContainer: RobotContainer? = null

    override fun robotInit() {
        robotContainer = RobotContainer(xbox)
    }

    override fun robotPeriodic() {}

    /** This function is called once when teleop is enabled. */
    override fun teleopInit() {}

    /** This function is called periodically during operator control. */
    override fun teleopPeriodic() {
        DriveCommand(drivetrain).schedule()
    }
    /** This function is called once when the robot is disabled. */
    override fun disabledInit() {
        drivetrain.stop()
    }

    /** This function is called periodically when disabled. */
    override fun disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    override fun testInit() {}

    /** This function is called periodically during test mode. */
    override fun testPeriodic() {}
}