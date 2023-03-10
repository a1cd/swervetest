package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandScheduler

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
     * robot container
     */
    private var robotContainer: RobotContainer? = null

    override fun robotInit() {
        robotContainer = RobotContainer(xbox)
    }

    override fun robotPeriodic() {
        robotContainer?.periodic()
        CommandScheduler.getInstance().run()
    }

    override fun simulationInit() {
        robotContainer = RobotContainer(xbox)

    }

    override fun simulationPeriodic() {
        robotContainer?.simulationPeriodic()
    }

    /** This function is called once when teleop is enabled. */
    override fun teleopInit() {
        robotContainer?.teleopInit()
    }

    /** This function is called periodically during operator control. */
    override fun teleopPeriodic() {
        robotContainer?.teleopPeriodic()
    }
    /** This function is called once when the robot is disabled. */
    override fun disabledInit() {
        robotContainer?.disabledInit()
    }

    /** This function is called periodically when disabled. */
    override fun disabledPeriodic() {
        robotContainer?.disabledPeriodic()
    }

    /** This function is called once when test mode is enabled. */
    override fun testInit() {
        robotContainer?.testInit()
    }

    /** This function is called periodically during test mode. */
    override fun testPeriodic() {
        robotContainer?.testPeriodic()
    }

    /** This function is called once when autonomous is enabled. */
    override fun autonomousInit() {
        robotContainer?.autonomousInit()
    }

    /** This function is called periodically during autonomous. */
    override fun autonomousPeriodic() {
        robotContainer?.autonomousPeriodic()
    }
}
