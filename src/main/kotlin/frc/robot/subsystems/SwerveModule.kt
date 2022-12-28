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
import kotlin.math.absoluteValue

class SwerveModule(
    private val name: String,
    driveId: Int,
    steerId: Int,
    encId: Int,
    val translation2d: Translation2d,
): SubsystemBase() {


    val driveMotor = WPI_TalonFX(driveId).apply { 
        configFactoryDefault()
    }

    val steerMotor = WPI_TalonFX(steerId).apply {
        configFactoryDefault()
    }

    val enc = CANCoder(encId).apply {
        configSensorDirection(false)
    }



    //should adjust these gains or characterize since they are a little slow
    val drivePid = PIDController(0.3,0.0,0.0)

    //should adjust these gains or characterize since they are a little slow
    val anglePid = PIDController(0.3,0.02,0.01).apply {
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
    fun move(drive: Double, angle: Double) {

        SmartDashboard.putNumber("${name} error", anglePid.positionError)
        SmartDashboard.putNumber("${name} ang", this.angle)

        driveMotor.set(drivePid.calculate(velocity,drive))
        if(drive.absoluteValue > 0.1) steerMotor.set(-MathUtil.clamp(anglePid.calculate( this.angle, angle), -0.5, 0.5))
        else steerMotor.stopMotor()

        driveMotor.setNeutralMode(m_brakeMode)
        steerMotor.setNeutralMode(m_brakeMode)
    }

    /**
     * @param swerveModuleState the state of the module
     */
    fun move(swerveModuleState: SwerveModuleState) {
        val optimizedState = SwerveModuleState.optimize(swerveModuleState, Rotation2d(angle) )

        SmartDashboard.putNumber("${name} desired speed", optimizedState.speedMetersPerSecond)
        SmartDashboard.putNumber("${name} desired ang", optimizedState.angle.degrees)
        SmartDashboard.putNumber("${name} ang", this.angle)
        SmartDashboard.putNumber("${name} vel", this.velocity)

        move(optimizedState.speedMetersPerSecond, optimizedState.angle.radians)
    }

    private var m_brakeMode = NeutralMode.Brake
    var brakeMode: Boolean
        get() = m_brakeMode == NeutralMode.Brake
        set(value) {
            driveMotor.setNeutralMode(if (value) NeutralMode.Brake else NeutralMode.Coast)
            steerMotor.setNeutralMode(if (value) NeutralMode.Brake else NeutralMode.Coast)
            m_brakeMode = if (value) NeutralMode.Brake else NeutralMode.Coast
        }
    fun zeroEncoders() {
        enc.position = 0.0
    }

    /**
     * put module in the default 0 rotation and 0 speed
     */
    fun reset() {
        this.move(SwerveModuleState(0.0, Rotation2d()))
    }

    // proper periodic method
    override fun periodic() {
        SmartDashboard.putNumber("${name} ang", this.angle)
        SmartDashboard.putNumber("${name} vel", this.velocity)
        super.periodic()
    }

    // proper simulation periodic method
    override fun simulationPeriodic() {
        SmartDashboard.putNumber("${name} ang", this.angle)
        SmartDashboard.putNumber("${name} vel", this.velocity)
        super.simulationPeriodic()
    }

    fun stop() {
        driveMotor.stopMotor()
        steerMotor.stopMotor()
    }
}