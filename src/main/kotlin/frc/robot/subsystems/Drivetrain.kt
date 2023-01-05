package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import frc.robot.Constants

/**
 * The drivetrain subsystem.
 */
class Drivetrain(
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
    )
): SimulatedSubsystem() {



    val modules = listOf(fl, fr, br, bl)
    override var children = modules.map { it as SimulatedSubsystem }

    val kinematics = SwerveDriveKinematics(
        *modules.map { it.translation2d }.toTypedArray()
    )

//     default command
//    override fun setDefaultCommand(defaultCommand: Command?) {
//        super.setDefaultCommand(DriveCommand(this))
//    }

    fun move(x: Double, y: Double, rotation: Double) {
        val speeds = ChassisSpeeds(x,y, rotation)
        val modules = kinematics.toSwerveModuleStates(speeds)
        // switch this to swerve kinematics, have one stick control X/Y, other for rotation
        modules.forEachIndexed { i, state ->
            this.modules[i].move(state)
        }
        println("moving drivetrain")
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

    override fun simulationPeriodic() {
        modules.forEach { it.simulationPeriodic() }
        // simulate battery voltage drop and roboRIO brownout
        // get correct voltage to roborio:
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
            *modules
                .flatMap { sequenceOf(it.driveMotorSystemSim, it.steerMotorSystemSim) }
                .map { it.currentDrawAmps }
                .toDoubleArray()
            )
        )
    }
}