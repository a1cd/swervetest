package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import frc.robot.Constants
import frc.robot.commands.DriveCommand
import frc.robot.controls.ControlScheme

/**
 * The drivetrain subsystem.
 */
class Drivetrain(
    controlScheme: ControlScheme,
    val fl: SwerveModule = SwerveModule(
        "fl",
        Constants.FrontLeftDriveMotor,
        Constants.FrontLeftSteerMotor,
        Constants.FrontLeftEncoder,
        Translation2d(.32, .32)
    ),
    val fr: SwerveModule = SwerveModule(
        "fr",
        Constants.FrontRightDriveMotor,
        Constants.FrontRightSteerMotor,
        Constants.FrontRightEncoder,
        Translation2d(.32, -.32)
    ),
    val bl: SwerveModule = SwerveModule(
        "bl",
        Constants.BackLeftDriveMotor,
        Constants.BackLeftSteerMotor,
        Constants.BackLeftEncoder,
        Translation2d(-.32, -.32)
    ),
    val br: SwerveModule = SwerveModule(
        "br",
        Constants.BackRightDriveMotor,
        Constants.BackRightSteerMotor,
        Constants.BackRightEncoder,
        Translation2d(-.32, .32)
    ),
): SimulatedSubsystem(fl,fr,br,bl) {

    val modules = listOf(fl, fr, br, bl)

    val kinematics = SwerveDriveKinematics(
        *modules.map { it.translation2d }.toTypedArray()
    )

//     default command
//    override fun setDefaultCommand(defaultCommand: Command?) {
//        super.setDefaultCommand(DriveCommand(this))
//    }

    fun move(x: Double, y: Double, rotation: Double) {
        val speeds = ChassisSpeeds(x,y, rotation)
        val moduleStates = kinematics.toSwerveModuleStates(speeds)
        // switch this to swerve kinematics, have one stick control X/Y, other for rotation
        moduleStates.forEachIndexed { i, state ->
            this.modules[i].target = state
        }
    }

    fun stop() {
        modules.forEach { it.stop() }
        println("stopping drivetrain")
    }

    fun zeroEncoders() {
        modules.forEach { it.zeroEncoders() }
        println("zeroing drivetrain encoders")
    }

    fun reset() {
        modules.forEach { it.reset() }
        println("resetting drivetrain")
    }

    var brakeMode: Boolean
        get() = modules.all { it.brakeMode }
        set(value) {
            modules.forEach { it.brakeMode = value }
        }

    // this sets the default command to DriveCommand(this)
    init {
        defaultCommand = DriveCommand(
            this,
            controlScheme
        )
    }

}