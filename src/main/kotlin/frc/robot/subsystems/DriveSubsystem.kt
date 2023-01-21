import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.ADXRS450_Gyro
import edu.wpi.first.wpilibj.interfaces.Gyro
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.DriveConstants
import frc.robot.subsystems.SwerveModule

class DriveSubsystem: SubsystemBase() {
    private val swerveTab = Shuffleboard.getTab("Swerve Diagnostics")

    //private PowerDistribution PDP = new PowerDistribution();
    private val xSpeedEntry = swerveTab.add("xBox xSpeed", 0)
        .entry
    private val ySpeedEntry = swerveTab.add("xBox ySpeed", 0)
        .entry
    private val rotEntry = swerveTab.add("xBox rot", 0)
        .entry
    private val frontLeftStateEntry = swerveTab.add("FL State v", 0)
        .entry
    private val frontRightStateEntry = swerveTab.add("FR State v", 0)
        .entry
    private val rearLeftStateEntry = swerveTab.add("RL State v", 0)
        .entry
    private val rearRightStateEntry = swerveTab.add("RR State v", 0)
        .entry
    private val gyroEntry = swerveTab.add("Gyro Heading", 0)
        .entry
    private val frontLeft = SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftTurningEncoderPorts,
        DriveConstants.kFrontLeftAngleZero
    )
    private val rearLeft = SwerveModule(
        DriveConstants.kRearLeftDriveMotorPort,
        DriveConstants.kRearLeftTurningMotorPort,
        DriveConstants.kRearLeftTurningEncoderPorts,
        DriveConstants.kRearLeftAngleZero
    )
    private val frontRight = SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightTurningEncoderPorts,
        DriveConstants.kFrontRightAngleZero
    )
    private val rearRight = SwerveModule(
        DriveConstants.kRearRightDriveMotorPort,
        DriveConstants.kRearRightTurningMotorPort,
        DriveConstants.kRearRightTurningEncoderPorts,
        DriveConstants.kRearRightAngleZero
    )

    // Initializing the gyro sensor
    private val gyro: Gyro = ADXRS450_Gyro()

    // Odometry class for tracking robot pose
    var odometry = SwerveDriveOdometry(DriveConstants.kDriveKinematics, gyro.rotation2d)
    override fun periodic() {
        // This method will be called once per scheduler run
        // Update the odometry in the periodic block
        gyroEntry.setDouble(gyro.angle)
        odometry.update(
            gyro.rotation2d,
            frontLeft.state,
            rearRight.state,
            frontRight.state,
            rearRight.state
        )
    }

    val pose: Pose2d
        // Returns the currently-estimated pose of the robot
        get() = odometry.poseMeters

    // Resets the odometry to the specified pose
    fun resetOdometry(pose: Pose2d?) {
        odometry.resetPosition(pose, gyro.rotation2d)
    }

    /**  Method to drive the robot using joystick info
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean) {
        val swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            if (fieldRelative) ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rot,
                gyro.rotation2d
            ) else ChassisSpeeds(xSpeed, ySpeed, rot)
        )
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        frontLeft.setDesiredState(swerveModuleStates[0])
        frontRight.setDesiredState(swerveModuleStates[1])
        rearLeft.setDesiredState(swerveModuleStates[2])
        rearRight.setDesiredState(swerveModuleStates[3])

        // Telemetry
        xSpeedEntry.setDouble(xSpeed)
        ySpeedEntry.setDouble(ySpeed)
        rotEntry.setDouble(rot)
        frontLeftStateEntry.setDouble(swerveModuleStates[0].speedMetersPerSecond)
        frontRightStateEntry.setDouble(swerveModuleStates[1].speedMetersPerSecond)
        rearRightStateEntry.setDouble(swerveModuleStates[2].speedMetersPerSecond)
        rearLeftStateEntry.setDouble(swerveModuleStates[3].speedMetersPerSecond)
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    fun setModuleStates(desiredStates: Array<SwerveModuleState?>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond
        )
        frontLeft.setDesiredState(desiredStates[0])
        frontRight.setDesiredState(desiredStates[1])
        rearLeft.setDesiredState(desiredStates[2])
        rearRight.setDesiredState(desiredStates[3])
    }

    // Resets the drive encoders to currently read a position of 0
    fun resetEncoders() {
        frontLeft.resetEncoders()
        rearLeft.resetEncoders()
        frontRight.resetEncoders()
        rearRight.resetEncoders()
    }

    // Zeroes the heading of the robot
    fun zeroHeading() {
        gyro.reset()
    }

    val heading: Double
        /**
         * Returns the heading of the robot.
         *
         * @return the robot's heading in degrees, from -180 to 180
         */
        get() = gyro.rotation2d.degrees
    val turnRate: Double
        /**
         * Returns the turn rate of the robot.
         *
         * @return The turn rate of the robot, in degrees per second
         */
        get() = gyro.rate * if (DriveConstants.kGyroReversed) -1.0 else 1.0
}