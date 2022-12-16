import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.CANCoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants.DRIVE_GEAR_RATIO
import frc.robot.Constants.WHEEL_CIRCUMFRENCE
import kotlin.math.absoluteValue

class SwerveModule(
    private val name: String,
    private val driveId: Int,
    private val steerId: Int,
    private val encId: Int,
    val translation2d: Translation2d,
) {

    val driveMotor = WPI_TalonFX(driveId).apply { 
        configFactoryDefault()
    }

    val steerMotor = WPI_TalonFX(steerId).apply {
        configFactoryDefault()
    }

    val enc = CANCoder(encId).apply {
        configSensorDirection(false)
        position = 0.0
    }



    //should adjust these gains or characterize since they are a little slow
    val drivePid = PIDController(0.3,0.0,0.0)

    //should adjust these gains or characterize since they are a little slow
    val anglePid = PIDController(0.3,0.02,0.01).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }
    val angle get() = MathUtil.angleModulus(Units.degreesToRadians(enc.position))

    val velocity get() = (driveMotor.selectedSensorVelocity/2048.0) * WHEEL_CIRCUMFRENCE * 10 / DRIVE_GEAR_RATIO

    // switch to taking a swerve module state instead of two doubles
    // angle here is in RADIANS
    fun move(drive: Double, angle: Double) {

        SmartDashboard.putNumber("${name} error", anglePid.positionError)
        SmartDashboard.putNumber("${name} ang", this.angle)

        driveMotor.set(drivePid.calculate(velocity,drive))
        if(drive.absoluteValue > 0.1) steerMotor.set(-MathUtil.clamp(anglePid.calculate( this.angle, angle), -0.5, 0.5))
        else steerMotor.stopMotor()
    }
    fun move(swerveModuleState: SwerveModuleState) {
        val optimizedState = SwerveModuleState.optimize(swerveModuleState, Rotation2d(angle) )

        SmartDashboard.putNumber("${name} desired speed", optimizedState.speedMetersPerSecond)
        SmartDashboard.putNumber("${name} desired ang", optimizedState.angle.degrees)
        SmartDashboard.putNumber("${name} ang", this.angle)
        SmartDashboard.putNumber("${name} vel", this.velocity)

        move(optimizedState.speedMetersPerSecond, optimizedState.angle.radians)
    }

}