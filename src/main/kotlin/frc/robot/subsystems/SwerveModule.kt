package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.CANCoder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.DRIVE_GEAR_RATIO
import frc.robot.Constants.WHEEL_CIRCUMFRENCE
import frc.robot.sim.PhysicsSim
import kotlin.math.absoluteValue

open class SwerveModule(
    val moduleName: String,
    val driveId: Int,
    val steerId: Int,
    val encId: Int,
    val translation2d: Translation2d,
): SubsystemBase() {

    val driveMotor = WPI_TalonFX(driveId).apply {
        configFactoryDefault()
        PhysicsSim.instance.addTalonFX(
            this,
            DRIVE_MOTOR_ACCELERATION,
            DRIVE_MOTOR_TOP_SPEED
        )
    }

    val steerMotor = WPI_TalonFX(steerId).apply {
        configFactoryDefault()
        PhysicsSim.instance.addTalonFX(
            this,
            TURN_MOTOR_ACCELERATION,
            TURN_MOTOR_TOP_SPEED
        )
    }

    val enc = CANCoder(encId).apply {
        configSensorDirection(false)
    }


    val DRIVE_P = 0.1
    val DRIVE_I = 0.01
    val DRIVE_D = 0.1

    //should adjust these gains or characterize since they are a little slow
    private val drivePid = PIDController(DRIVE_P, DRIVE_I, DRIVE_D)
    val ANGLE_P = 0.1
    val ANGLE_I = 0.01
    val ANGLE_D = 0.1
    //should adjust these gains or characterize since they are a little slow
    val anglePid = PIDController(ANGLE_P, ANGLE_I, ANGLE_D).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }

    val angle get() = MathUtil.angleModulus(Units.degreesToRadians(enc.position))

    val velocity get() = (driveMotor.selectedSensorVelocity/2048.0) * WHEEL_CIRCUMFRENCE * 10 / DRIVE_GEAR_RATIO

    // switch to taking a DriveSubsystem module state instead of two doubles
    // angle here is in RADIANS
    /**
     * @param drive the speed of the module
     * @param angle angle in radians of the module
     */
    open fun move(drive: Double, angle: Double) {

        SmartDashboard.putNumber("$moduleName error", anglePid.positionError)
        SmartDashboard.putNumber("$moduleName ang", this.angle)

        driveMotor.set(drivePid.calculate(velocity,drive))
        if(drive.absoluteValue > 0.1) steerMotor.set(-MathUtil.clamp(anglePid.calculate( this.angle, angle), -0.5, 0.5))
        else steerMotor.stopMotor()

        driveMotor.setNeutralMode(m_brakeMode)
        steerMotor.setNeutralMode(m_brakeMode)
    }

    /**
     * @param swerveModuleState the state of the module
     */
    open fun move(swerveModuleState: SwerveModuleState) {
        val optimizedState = SwerveModuleState.optimize(swerveModuleState, Rotation2d(angle) )

        SmartDashboard.putNumber("${moduleName} desired speed", optimizedState.speedMetersPerSecond)
        SmartDashboard.putNumber("${moduleName} desired ang", optimizedState.angle.degrees)
        SmartDashboard.putNumber("${moduleName} ang", this.angle)
        SmartDashboard.putNumber("${moduleName} vel", this.velocity)

        move(optimizedState.speedMetersPerSecond, optimizedState.angle.radians)
    }

    private var m_brakeMode = NeutralMode.Brake
    open var brakeMode: Boolean
        get() = m_brakeMode == NeutralMode.Brake
        set(value) {
            driveMotor.setNeutralMode(if (value) NeutralMode.Brake else NeutralMode.Coast)
            steerMotor.setNeutralMode(if (value) NeutralMode.Brake else NeutralMode.Coast)
            m_brakeMode = if (value) NeutralMode.Brake else NeutralMode.Coast
        }
    open fun zeroEncoders() {
        enc.position = 0.0
    }

    /**
     * put module in the default 0 rotation and 0 speed
     */
    open fun reset() {
        this.move(SwerveModuleState(0.0, Rotation2d()))
    }

    // proper periodic method
    override fun periodic() {
        SmartDashboard.putNumber("${moduleName} ang", this.angle)
        SmartDashboard.putNumber("${moduleName} vel", this.velocity)
        super.periodic()
    }

    // proper simulation periodic method
    override fun simulationPeriodic() {
        SmartDashboard.putNumber("${moduleName} ang", this.angle)
        SmartDashboard.putNumber("${moduleName} vel", this.velocity)
        super.simulationPeriodic()
    }

    open fun stop() {
        driveMotor.stopMotor()
        steerMotor.stopMotor()
    }

    override fun toString(): String {
        // debug info
        return("position: ${enc.position}" +
                "velocity: ${driveMotor.selectedSensorVelocity}" +
                "angle: ${angle}" +
                "drive: ${driveMotor.get()}" +
                "steer: ${steerMotor.get()}" +
                "power: ${driveMotor.getMotorOutputPercent()}" +
                "driveMotorcurrent: ${driveMotor.getOutputCurrent()}")
    }
    companion object {
        // add to simulation
        // Constants for turn motor acceleration and top speed
        // Note: These values are rough estimates and the actual performance of the motors may vary
        const val TURN_MOTOR_ACCELERATION = 2.5 // time to reach full speed, in seconds
        const val TURN_MOTOR_TOP_SPEED = 60.0 // maximum motor velocity, in ticks per 100ms

        // Constants for drive motor acceleration and top speed
        // Note: These values are rough estimates and the actual performance of the motors may vary
        const val DRIVE_MOTOR_ACCELERATION = 2.5 // time to reach full speed, in seconds
        const val DRIVE_MOTOR_TOP_SPEED = 80.0 // maximum motor velocity, in ticks per 100ms
    }

}