package frc.robot

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import kotlin.math.PI

object Constants {
    object DriveConstants {
        const val kFrontLeftDriveMotorPort = 8
        const val kRearLeftDriveMotorPort = 2
        const val kFrontRightDriveMotorPort = 4
        const val kRearRightDriveMotorPort = 6
        const val kFrontLeftTurningMotorPort = 1
        const val kRearLeftTurningMotorPort = 3
        const val kFrontRightTurningMotorPort = 5
        const val kRearRightTurningMotorPort = 7
        const val kFrontLeftTurningEncoderPorts = 9
        const val kRearLeftTurningEncoderPorts = 10
        const val kFrontRightTurningEncoderPorts = 11
        const val kRearRightTurningEncoderPorts = 12
        const val kFrontLeftAngleZero = 146.074 //-32.959
        const val kRearLeftAngleZero = 152.842 //-28.477
        const val kFrontRightAngleZero = 82.617
        const val kRearRightAngleZero = -10.547 //169.805
        const val kFrontLeftTurningEncoderReversed = false
        const val kRearLeftTurningEncoderReversed = true
        const val kFrontRightTurningEncoderReversed = false
        const val kRearRightTurningEncoderReversed = true
        const val kFrontLeftDriveEncoderReversed = false
        const val kRearLeftDriveEncoderReversed = true
        const val kFrontRightDriveEncoderReversed = false
        const val kRearRightDriveEncoderReversed = true
        const val kTrackWidth = 0.5969 // meters

        // Distance between centers of right and left wheels on robot
        const val kWheelBase = 0.5969 // meters

        // Distance between front and back wheels on robot
        val kDriveKinematics = SwerveDriveKinematics(
            Translation2d(kWheelBase / 2, kTrackWidth / 2),
            Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        )
        const val kGyroReversed = false

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        const val ksVolts = 0.509
        const val kvVoltSecondsPerMeter = 2.73
        const val kaVoltSecondsSquaredPerMeter = 0.124
        const val kMaxSpeedMetersPerSecond = 3.0
        const val kMaxRotationalSpeedMetersPerSecond = 4.0 // Constant multiplied by controller input
        const val ksTurning = 0.7
        const val kvTurning = 0.216
        const val kMaxAngularSpeedRadiansPerSecond = Math.PI * 2
    }
    object ModuleConstants {
        // Drive motor -> FX Encoder (2048 units)
        // Turning motor -> CTRE CANcoder (4096 units)
        const val kMaxModuleAngularSpeedRadiansPerSecond = 20 * Math.PI
        const val kMaxModuleAngularAccelerationRadiansPerSecondSquared = 35 * Math.PI
        const val kDriveGearRatio = 8.14 // Todo:
        const val kTurningGearRatio = 12.8
        const val kDriveFXEncoderCPR = 2048
        const val kTurningCANcoderCPR = 4096
        const val kWheelDiameterMeters = 0.1016 // 4 inches
        const val kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI // C = D * pi
        const val kDrivetoMetersPerSecond = 10 * kWheelCircumferenceMeters / (kDriveGearRatio * 2048)

        //PID turn motor values
        const val kPModuleTurningController = 6.0
        const val kDModuleTurningController = .1
        const val kPModuleDriveController = 3.0
    }

    // <-----> Drivetrain CAN IDs <----->
    const val FrontLeftDriveMotor = 10
    const val FrontLeftSteerMotor = 14
    const val FrontLeftEncoder = 6
    const val FrontRightDriveMotor = 11
    const val FrontRightSteerMotor = 15
    const val FrontRightEncoder = 7
    const val BackRightDriveMotor = 12
    const val BackRightSteerMotor = 16
    const val BackRightEncoder = 8
    const val BackLeftDriveMotor = 13
    const val BackLeftSteerMotor = 17
    const val BackLeftEncoder = 9

    // <-----> Drivetrain Constants <----->
    const val WHEEL_RADIUS = .0508
    val WHEEL_CIRCUMFRENCE = WHEEL_RADIUS*2* PI
    const val DRIVE_GEAR_RATIO = 6.75
}