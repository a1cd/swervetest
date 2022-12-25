package frc.robot

import kotlin.math.PI

object Constants {
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