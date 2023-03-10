package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.robot.commands.DriveCommand
import frc.robot.commands.ResetCommand
import frc.robot.commands.ToggleBrakemodeCommand
import frc.robot.commands.ZeroEncodersCommand
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.Drivetrain

class RobotContainer(
    xbox: XboxController = XboxController(0)
) {
    var state: RobotState? = null

    val controlType = DefaultControlScheme(xbox)

    val drivetrain = Drivetrain()

    var controlScheme = controlType.apply {
        // control scheme
        zeroEncoders.whileActiveOnce(ZeroEncodersCommand(drivetrain))
        toggleBrakeMode.whileActiveOnce(ToggleBrakemodeCommand(drivetrain))
        resetAll.whileActiveOnce(ResetCommand(drivetrain))

        // drive command
        forewardThresholdTrigger
            .or(strafeThresholdTrigger)
            .or(rotationThresholdTrigger)
            .whileActiveContinuous(RunCommand({
                DriveCommand(drivetrain, this).execute()
            }, drivetrain))
        // this actually took forever to fix
        // it was a problem with the control scheme, it was not sending updated values
        // to the drive command so it was always using the starting values
        // i fixed it by making the drive command take in the control scheme as a parameter
    }

    /**
     * called by robot periodic
     */
    fun periodic() {
        // put the state in the smart dashboard along with control scheme data
        state?.let { state ->
            SmartDashboard.putString("state", state.name)
        }
        SmartDashboard.putNumber("forward", controlScheme.forward)
        SmartDashboard.putNumber("strafe", controlScheme.strafe)
        SmartDashboard.putNumber("rotation", controlScheme.rotation)
        SmartDashboard.putBoolean("brakeMode", drivetrain.brakeMode)
    }
    fun disabledInit() {
        state = RobotState.DISABLED
        drivetrain.stop()
    }
    fun disabledPeriodic() {}
    fun autonomousInit() {
        state = RobotState.AUTONOMOUS
    }
    fun autonomousPeriodic() {
    }
    fun teleopInit() {
        state = RobotState.TELEOP
    }
    fun teleopPeriodic() {
        controlScheme
    }
    fun testInit() {
        state = RobotState.TEST
    }
    fun testPeriodic() {}
    fun simulationPeriodic() {
    }
}