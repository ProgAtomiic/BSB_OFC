package frc.robot.commands.teleop.Reef;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ResetLevel extends Command {
  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;

  public ResetLevel(ElevatorSubsystem elevator, ArmSubsystem arm) {
    this.elevator = elevator;
    this.arm = arm;
    addRequirements(elevator, arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    ArmSubsystem.angleset(-1);

    if (ArmSubsystem.angleget() < 10) {
      double posicaoAtual = ElevatorSubsystem.GetPosicaoElevador();

      if (Math.abs(posicaoAtual) < 0.08) {
        ElevatorSubsystem.DesligarElevador();
      } else {
        ElevatorSubsystem.PIDNoFFMaisFF(0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.set(0);
    ElevatorSubsystem.LigarMotorArm(0);
}

  @Override
  public boolean isFinished() {
    return ArmSubsystem.angleget() < 5 && Math.abs(ElevatorSubsystem.GetPosicaoElevador()) < 0.08;
  }
}
