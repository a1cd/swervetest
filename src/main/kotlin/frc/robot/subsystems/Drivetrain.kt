package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.DriveCommand
import frc.robot.Constants

class Drivetrain: SubsystemBase() {
    val fl = SwerveModule(
        "fl",
        Constants.FrontLeftDriveMotor,
        Constants.FrontLeftSteerMotor,
        Constants.FrontLeftEncoder,
        Translation2d(.32, .32)
    )
    val fr = SwerveModule(
        "fr",
        Constants.FrontRightDriveMotor,
        Constants.FrontRightSteerMotor,
        Constants.FrontRightEncoder,
        Translation2d(.32, -.32)
    )
    val bl = SwerveModule(
        "bl",
        Constants.BackLeftDriveMotor,
        Constants.BackLeftSteerMotor,
        Constants.BackLeftEncoder,
        Translation2d(-.32, -.32)
    )
    val br = SwerveModule(
        "br",
        Constants.BackRightDriveMotor,
        Constants.BackRightSteerMotor,
        Constants.BackRightEncoder,
        Translation2d(-.32, .32)
    )

    val modules = arrayOf(fl, fr, br, bl)

    val kinematics = SwerveDriveKinematics(
        *modules.map { it.translation2d }.toTypedArray()
    )

    // default command
    override fun setDefaultCommand(defaultCommand: Command?) {
        super.setDefaultCommand(DriveCommand(this))
    }

    fun move(x: Double, y: Double, rotation: Double) {
        val speeds = ChassisSpeeds(x,y, rotation)
        val modules = kinematics.toSwerveModuleStates(speeds)
        // switch this to swerve kinematics, have one stick control X/Y, other for rotation
        modules.forEachIndexed { i, state ->
            this.modules[i].move(state)
        }
    }

    fun stop() {
        modules.forEach { it.stop() }
    }

    fun zeroEncoders() {
        modules.forEach { it.zeroEncoders() }
    }

    fun reset() {
        modules.forEach { it.reset() }
    }

    var brakeMode: Boolean
        get() = modules.all { it.brakeMode }
        set(value) {
            modules.forEach { it.brakeMode = value }
        }

    override fun periodic() {
        modules.forEach { it.periodic() }
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}