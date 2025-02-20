package frc.robot.commands.teleop;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class arm_command extends Command {
  private final ArmSubsystem arm;
  private final XboxController Controle_0;
  public double angulo;

  public arm_command(ArmSubsystem arm, XboxController Controle_0, int posicao) {

    this.arm = arm;
    this.Controle_0 = Controle_0;
    addRequirements(arm);
    angulo = posicao;

  }
  // Called when the command is initially scheduled.

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (angulo == -10) {
      arm.set(-0.1);
    } else {
      arm.angleset(angulo);

      System.out.print("ALVO");
      System.out.println(angulo);
      System.out.print("ATUAL");
      System.out.println(arm.angleget() * 14.4);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.stoparm();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
