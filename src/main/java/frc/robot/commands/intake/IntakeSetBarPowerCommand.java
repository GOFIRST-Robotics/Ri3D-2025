package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeSetBarPowerCommand extends Command {
    private final double power;

    public IntakeSetBarPowerCommand(double power) {
        this.power = power;
        addRequirements(Robot.m_intakeSubsystem);
    }

    @Override
    public void execute() {
        Robot.m_intakeSubsystem.setIntakeBarPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.m_intakeSubsystem.stopIntakeBar();
    }
}
