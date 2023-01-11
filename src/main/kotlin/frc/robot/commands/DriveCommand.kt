package frc.robot.commands

import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.controls.ControlScheme
import frc.robot.subsystems.Drivetrain
import kotlin.math.pow
import kotlin.math.withSign


/**
 * The default drive command.
 */
class DriveCommand(
    val drivetrain: Drivetrain,
    val controlScheme: ControlScheme
): CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        drivetrain.move(
            applyDeadband(controlScheme.forward , 0.055).pow(2.0).withSign(controlScheme.forward),
            applyDeadband(controlScheme.strafe,   0.055).pow(2.0).withSign(controlScheme.strafe),
            applyDeadband(controlScheme.rotation, 0.055).pow(2.0).withSign(controlScheme.rotation)
        )
    }
}