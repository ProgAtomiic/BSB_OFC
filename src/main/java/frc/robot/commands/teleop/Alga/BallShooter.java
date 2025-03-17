package frc.robot.commands.teleop.Alga;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaSubsystem;


public class BallShooter extends Command {
  double linhacomalga = -45;
  private final AlgaSubsystem shooterAlga;
  double TempoSeguranca;
  boolean Terminado;

  static ArrayList<Integer> parametros = new ArrayList<>();

  public BallShooter(AlgaSubsystem ShooterAlga) {
    this.shooterAlga = ShooterAlga;
    addRequirements(shooterAlga);
  }


  @Override
  public void initialize() {
    TempoSeguranca = Timer.getFPGATimestamp();
    linhacomalga = -45;
  }

  @Override
  public void execute() {


    if (Timer.getFPGATimestamp() - TempoSeguranca > 2.5) {
      AlgaSubsystem.linha(0);
      AlgaSubsystem.desligaintake();
    } else {
      if (AlgaSubsystem.TemAlga() == true) {
        AlgaSubsystem.linha(linhacomalga);
        if (AlgaSubsystem.EncoderLinha() > (linhacomalga - 5) && AlgaSubsystem.EncoderLinha() < (linhacomalga + 5)) {
          AlgaSubsystem.ligaintake(0.8);
        } else {
          AlgaSubsystem.ligaintake(0);
        }
      } else {
        AlgaSubsystem.linha(0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    AlgaSubsystem.linha(0);
    AlgaSubsystem.desligaintake();
  }

  @Override
  public boolean isFinished() {
    // return Controle_0.getRightTriggerAxis() < 0.25;//|| Terminado == true;
    return false;
  }
}
