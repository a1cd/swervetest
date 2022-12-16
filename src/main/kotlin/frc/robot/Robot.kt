package frc.robot

import SwerveModule
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.XboxController

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {

    val xbox = XboxController(0)

    val fl = SwerveModule("fl", 10, 14, 6, Translation2d(.32,.32))
    val fr = SwerveModule("fr", 11, 15, 7, Translation2d(.32,-.32))
    val br = SwerveModule("br", 12, 16, 8, Translation2d(-.32,-.32))
    val bl = SwerveModule("bl", 13, 17, 9, Translation2d(-.32,.32))

    val modules = arrayOf(fl, fr, br, bl)

    val kinematics = SwerveDriveKinematics(*modules.map { it.translation2d }.toTypedArray())



    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {}

    override fun robotPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    override fun teleopInit() {}

    /** This function is called periodically during operator control. */
    override fun teleopPeriodic() {
        val speeds = ChassisSpeeds(-xbox.leftY,-xbox.leftX, -xbox.rightX)
        val modules = kinematics.toSwerveModuleStates(speeds)
        // switch this to swerve kinematics, have one stick control X/Y, other for rotation
        modules.forEachIndexed({i,state ->
            this.modules[i].move(state)
        })
    }

    /** This function is called once when the robot is disabled. */
    override fun disabledInit() {}

    /** This function is called periodically when disabled. */
    override fun disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    override fun testInit() {}

    /** This function is called periodically during test mode. */
    override fun testPeriodic() {}
}
