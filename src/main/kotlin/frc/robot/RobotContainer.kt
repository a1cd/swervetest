package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.robot.commands.DriveCommand
import frc.robot.controls.DefaultControlScheme
import frc.robot.subsystems.Drivetrain

class RobotContainer(
    xbox: XboxController = XboxController(0)
) {
    var drivetrain = Drivetrain()

    // control scheme
    val controlScheme = DefaultControlScheme(xbox)

    init {
        configureBindings()
    }
    private fun configureBindings() {
        controlScheme.zeroEncoders.whenPressed(
            RunCommand({ drivetrain.zeroEncoders() }, drivetrain)
        )
        controlScheme.toggleBrakeMode.whenPressed(
            RunCommand({ drivetrain.brakeMode = !drivetrain.brakeMode }, drivetrain)
        )
        controlScheme.resetAll.whenPressed(
            RunCommand({ drivetrain.reset() }, drivetrain)
        )
        controlScheme
            .forewardThresholdTrigger
            .or(controlScheme.strafeThresholdTrigger)
            .or(controlScheme.rotationThresholdTrigger)
            .whileActiveContinuous(DriveCommand(
                drivetrain,
                controlScheme.forward,
                controlScheme.strafe,
                controlScheme.rotation
            ))
    }
}