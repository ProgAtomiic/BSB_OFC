package frc.robot.commands.teleop.Alga;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaSubsystem;

public class BallSetPoint extends Command {
  double linhacomalga = -45;
  private final AlgaSubsystem shooterAlga;
  double TempoSeguranca;
  boolean Terminado;

  static ArrayList<Integer> parametros = new ArrayList<>();

  public BallSetPoint(AlgaSubsystem ShooterAlga) {
    this.shooterAlga = ShooterAlga;
    addRequirements(shooterAlga);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (AlgaSubsystem.TemAlga() == false) {
      if (AlgaSubsystem.EncoderLinha() < 5 && AlgaSubsystem.EncoderLinha() > -5) {
        AlgaSubsystem.desligalinha();
      } else {
        AlgaSubsystem.linha(0);
      }
    } else {
      AlgaSubsystem.linha(linhacomalga);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
