package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

class AlignModules(
    val drivetrain: Drivetrain
): CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        drivetrain.setOffsetToForeward()
    }

    override fun isFinished(): Boolean {
        return true
    }
}