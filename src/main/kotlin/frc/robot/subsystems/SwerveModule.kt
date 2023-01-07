package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.CANCoder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants.DRIVE_GEAR_RATIO
import frc.robot.Constants.WHEEL_CIRCUMFRENCE
import frc.robot.Constants.WHEEL_RADIUS
import frc.robot.sim.PhysicsSim
import kotlin.math.abs

/**
 * The drivetrain subsystem.
 */
open class SwerveModule(
    val moduleName: String,
    val driveId: Int,
    val steerId: Int,
    val encId: Int,
    val translation2d: Translation2d,
): SimulatedSubsystem() {



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
//        PhysicsSim.instance.addCANCoder(this)
    }



    //should adjust these gains or characterize since they are a little slow
    val DRIVE_P = 0.1
    val DRIVE_I = 0.0
    val DRIVE_D = 0.0
    val drivePid = PIDController(DRIVE_P, DRIVE_I, DRIVE_D).apply {
        setTolerance(0.1)
    }
    //should adjust these gains or characterize since they are a little slow
    val ANGLE_P = 0.01
    val ANGLE_I = 0.0
    val ANGLE_D = 0.0
    val anglePid = PIDController(ANGLE_P, ANGLE_I, ANGLE_D).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }
    // pid calc software came up with the optimal P I and D values by driving the robot around and measuring stuff
    var target = SwerveModuleState()
        get() = field
        set(value) {
            field = value
            drivePid.setpoint = value.speedMetersPerSecond
            anglePid.setpoint = value.angle.radians
        }
    val state: SwerveModuleState
        get() = SwerveModuleState(
            // convert encoder ticks to meters
            if (isSim) driveMotorSystemSim.angularVelocityRadPerSec * WHEEL_RADIUS
            else driveMotor.selectedSensorVelocity / 2048 * WHEEL_CIRCUMFRENCE,
            Rotation2d(
                if (isSim) -MathUtil.angleModulus(steerMotorSystemSim.angularPositionRad)
                else MathUtil.angleModulus(Units.degreesToRadians(enc.position/4096.0*360.0))
            )
        )
    var angle: Double = 0.0

    var velocity: Double = 0.0
    val driveMotorSystemSim: DCMotorSim by lazy {
        DCMotorSim(
            // gearbox is just the DCMotor that runs the wheel
            // we use a falcon 500 for the drive motor
            DCMotor.getFalcon500(1),
            // gearing is the ratio of the wheel to the motor
            // our ratio is defined in the constants file
            DRIVE_GEAR_RATIO,
            // jKgMetersSquared is the moment of inertia of the wheel and gearbox and robot torque with gravity.
            // just a guess for now
            0.002825
        )
    }
    val steerMotorSystemSim: DCMotorSim by lazy {
        DCMotorSim(
            // we use a falcon 500 for the steer motor
            DCMotor.getFalcon500(1),
            // gearing is the ratio of the steer axle to the motor
            // our ratio is not defined in the constants file but it is 1:1
            1.0,
            // jKgMetersSquared is the moment of inertia of the steer gears, the swerve wheel and all the spinning parts of
            // the module.
            // just a guess for now
            0.000156
        )
    }

    private fun setMotors(drive: Double, steer: Double) {
        // simulate/set the motors
        if (isSim) {
            // limit to stop destabilizing the simulation
            steerMotorSystemSim.setInputVoltage((steer * RoboRioSim.getVInVoltage()).coerceIn(-12.0, 12.0))
            driveMotorSystemSim.setInputVoltage((drive * RoboRioSim.getVInVoltage()).coerceIn(-12.0, 12.0))
        } else {
            steerMotor.set(steer)
            driveMotor.set(drive)
        }
    }
    // switch to taking a DriveSubsystem module state instead of two doubles
    // angle here is in RADIANS
    /**
     * @param drive the speed of the module
     * @param angle angle in radians of the module
     */
    open fun move(drive: Double, angle: Double) {
        SmartDashboard.putNumber("$moduleName error", anglePid.positionError)
        SmartDashboard.putNumber("$moduleName ang3", this.angle)
        setMotors(
            drivePid.calculate(velocity),
            -MathUtil.clamp(anglePid.calculate(this.angle), -0.25, 0.25)
        )
        if (drive<0.05) {
            stop()
        }

        brakeMode = abs(anglePid.positionError) < 0.05 && abs(drivePid.positionError) < 0.05
        // set brake mode to true if the module close to target angle and speed (brake mode does not engage brakes, just
        // stops the motor from freely spinning when pushed)
        throw RuntimeException("bad function")
    }

    /**
     * @param swerveModuleState the state of the module
     */
    open fun move(swerveModuleState: SwerveModuleState) {
        val drivePower = drivePid.calculate(velocity, swerveModuleState.speedMetersPerSecond)
        // fix this
        // ok here is the commented fixed code (it now makes the motors reach and stay at the target speed):
        /*

        val drivePower = drivePid.calculate(
            velocity,
            swerveModuleState.speedMetersPerSecond,
            1.0 / 50.0
        )
         */
        val steerPower = -MathUtil.clamp(anglePid.calculate(angle), -1.0, 1.0)
        // smartdashboard
        SmartDashboard.putNumber("$moduleName desired speed", drivePid.setpoint)
        SmartDashboard.putNumber("$moduleName desired ang", anglePid.setpoint)
        SmartDashboard.putNumber("$moduleName drive power", drivePower)
        SmartDashboard.putNumber("$moduleName steer power", steerPower)
        setMotors(
            drivePower,
            steerPower
        )
    }
    private fun move() {
        move(target)
    }

    private var mBrakemode = NeutralMode.Brake
    open var brakeMode: Boolean
        get() = mBrakemode == NeutralMode.Brake
        set(value) {
            driveMotor.setNeutralMode(if (value) NeutralMode.Brake else NeutralMode.Coast)
            steerMotor.setNeutralMode(if (value) NeutralMode.Brake else NeutralMode.Coast)
            mBrakemode = if (value) NeutralMode.Brake else NeutralMode.Coast
        }
    open fun zeroEncoders() =
        if (isSim) steerMotor.selectedSensorPosition = 0.0
        else enc.position = 0.0

    /**
     * put module in the default 0 rotation and 0 speed
     */
    open fun reset() {
        target = SwerveModuleState()
    }

    // proper periodic method
    override fun periodic() {
        // update velocity and angle
        velocity = state.speedMetersPerSecond
        angle = state.angle.radians
        // update the pid controller with the current state of the module
        move()

        SmartDashboard.updateValues()
    }

    override var lastTime = 0.0
    override val currentDraw: Double
        get() {
            return if (isSim) driveMotorSystemSim.currentDrawAmps + steerMotorSystemSim.currentDrawAmps
            // else driveMotor.outputCurrent + steerMotor.outputCurrent
            // deprecated outputCurrent, was replaced with supplyCurrent and statorCurrent
            // the difference between the two is that supplyCurrent is the current going into the motor controller
            // and statorCurrent is the current going into the motor
            // we want the current going into the motor
            else driveMotor.statorCurrent + steerMotor.statorCurrent
        }
    override fun simUpdate(dt: Double) {
        if (isSim) {
            steerMotorSystemSim.update(dt)
            driveMotorSystemSim.update(dt)
        }
        SmartDashboard.putNumber("$moduleName ang", this.angle)
        SmartDashboard.putNumber("$moduleName vel", this.velocity)
    }
    open fun stop() {
        setMotors(0.0, 0.0)
        driveMotor.stopMotor()
        steerMotor.stopMotor()
        brakeMode = true
    }

    override fun toString(): String {
        // debug info
        return("position: ${enc.position}" +
                "velocity: ${driveMotor.selectedSensorVelocity}" +
                "angle: $angle" +
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