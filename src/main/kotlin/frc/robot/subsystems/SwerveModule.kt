// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.ctre.phoenix.sensors.CANCoderStatusFrame
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.ModuleConstants
import frc.robot.Constants.ModuleConstants.kDrivetoMetersPerSecond

class SwerveModule(
    driveMotorChannel: Int,
    turningMotorChannel: Int,
    turningEncoderPorts: Int,
    angleZero: Double
) : SubsystemBase() {
    private val driveMotor = WPI_TalonFX(driveMotorChannel)
    private val turningMotor = WPI_TalonFX(turningMotorChannel)
    private val turnEncoder: CANCoder

    // Driving encoder uses the integrated FX encoder
    // e.g. testMotor.getSelectedSensorPosition();
    // PID controller for velocity. DO NOT SET kD.  It is redundant as setVoltage() already controls this
    private val drivePIDController: PIDController = PIDController(ModuleConstants.kPModuleDriveController, 0.0, 0.0)

    //Using a TrapezoidProfile PIDController to allow for smooth turning
    private val turningPIDController: ProfiledPIDController = ProfiledPIDController(
        ModuleConstants.kPModuleTurningController,
        0.0,
        ModuleConstants.kDModuleTurningController,
        TrapezoidProfile.Constraints(
            ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
            ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared
        )
    )
//    var driveFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(
//        DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter
//    )
//    var turnFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(
//        DriveConstants.ksTurning, DriveConstants.kvTurning
//    )

    init {
        // Configure current limits for motors - prevents disabling/brownout
        driveMotor.configSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40.0, 45.0, 0.5)) //40, 45, 0.5
        //driveMotor.configClosedloopRamp(0.25);
        driveMotor.configStatorCurrentLimit(StatorCurrentLimitConfiguration(true, 40.0, 45.0, 0.5))
        driveMotor.setNeutralMode(NeutralMode.Brake)

        // Configure the encoders for both motors
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0)
        turnEncoder = CANCoder(turningEncoderPorts)
        turnEncoder.configMagnetOffset(-1 * angleZero)
        turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180)
        turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100)
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255)
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255)
        turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255)
        turningMotor.setNeutralMode(NeutralMode.Brake)
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI)
    }

    val state: SwerveModuleState
        get() = SwerveModuleState(
            kDrivetoMetersPerSecond * driveMotor.selectedSensorVelocity,
            Rotation2d(2 * Math.PI / 360 * turnEncoder.absolutePosition)
        )

    fun setDesiredState(desiredState: SwerveModuleState?) {
        //Optimize the reference state to avoid spinning further than 90 degrees
        val state: SwerveModuleState = SwerveModuleState.optimize(desiredState, this.state.angle)

        //Calculate the drive output from the drive PID controller
        val driveOutput: Double = (drivePIDController.calculate(this.state.speedMetersPerSecond, state.speedMetersPerSecond)
                )//+ driveFeedforward.calculate(state.speedMetersPerSecond))
        val turnOutput: Double = (turningPIDController.calculate(this.state.angle.radians, state.angle.radians)
                )//+ turnFeedForward.calculate(turningPIDController.setpoint.velocity))

        // Calculate the turning motor output from the turning PID controller
        driveMotor.setVoltage(driveOutput)
        turningMotor.setVoltage(turnOutput)
    }

    fun resetEncoders() {
        turnEncoder.position = 0.0
        driveMotor.selectedSensorPosition = 0.0
    }

    override fun periodic() {
        // This method will be called once per scheduler run
    }
}