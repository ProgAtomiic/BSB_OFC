package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.commands.teleop.Reef.AlinhamentoReef;
import frc.robot.commands.teleop.Reef.L1;
import frc.robot.commands.teleop.Reef.L2;
import frc.robot.commands.teleop.Reef.L3;
import frc.robot.commands.teleop.Reef.L4;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LevelSet {
  private String LadoEscolhido = "";
  private int LevelEscolhido = 0;
  private boolean ComandoLevelIniciou = false;
  private boolean AlinhamentoTerminado = false;
  private boolean LevelTerminado = false;

  private boolean Level2;
  private boolean Parte1;

  private ElevatorSubsystem Elevator;
  private SwerveSubsystem Swerve;
  private ArmSubsystem armSubsystem;

  

  private XboxController Controller;

  public LevelSet(ElevatorSubsystem Elevator, SwerveSubsystem Swerve, ArmSubsystem armSubsystem,
      XboxController Controller) {
    this.Elevator = Elevator;
    this.Swerve = Swerve;
    this.armSubsystem = armSubsystem;
    this.Controller = Controller;
  }

  public void selectSide(String side) {
    if (LadoEscolhido.isEmpty()) { // Prevent changing the side once selected
      LadoEscolhido = side;

      if (side.equals("Esquerda")) {
        Controller.setRumble(RumbleType.kLeftRumble, 1.0);

      } else if (side.equals("Direita")) {
        Controller.setRumble(RumbleType.kRightRumble, 1.0);
      }
      // System.out.println(LadoEscolhido);

    }
  }

  public void selectLevel(int level) {
    if (!LadoEscolhido.isEmpty() && LevelEscolhido == 0) {
      LevelEscolhido = level;
      Controller.setRumble(RumbleType.kLeftRumble, 0);
      Controller.setRumble(RumbleType.kRightRumble, 0);
      // System.out.println("level escolhido" + LevelEscolhido);

    }
  }

  public String getLadoEscolhido() {
    return LadoEscolhido;
  }

  public int getLevelEscolhido() {
    return LevelEscolhido;
  }

  public boolean isComandoLevelIniciou() {
    return ComandoLevelIniciou;
  }

  public void setComandoLevelIniciou(boolean started) {
    ComandoLevelIniciou = started;
  }

  public boolean isAlinhamentoTerminado() {
    return AlinhamentoTerminado;
  }

  public void setAlinhamentoTerminado(boolean finished) {
    AlinhamentoTerminado = finished;
  }

  public boolean isLevelTerminado() {
    return LevelTerminado;
  }

  public void setLevelTerminado(boolean finished) {
    LevelTerminado = finished;
  }

  public boolean Terminado() {
    return LevelTerminado;
  }

  public void Terminado(boolean finished) {
    LevelTerminado = finished;
  }

  public boolean PodeAlinhar() {
    if (!LadoEscolhido.isEmpty() && LevelEscolhido != 0) {
      return true; // Notify RobotContainer that alignment should start
    } else {
      return false;
    }
  }

  public void resetSelection() {
    LadoEscolhido = "";
    LevelEscolhido = 0;
    ComandoLevelIniciou = false;
    AlinhamentoTerminado = false;

    Controller.setRumble(RumbleType.kLeftRumble, 0);
    Controller.setRumble(RumbleType.kRightRumble, 0);
  }

  public void setLevel2(boolean L2) {
    Level2 = L2;
  }

  public boolean isLevel2() {
    return Level2;
  }

    public void setParte1Terminado(boolean Terminado) {
    Parte1 = Terminado;
  }

  public boolean isParte1Terminado() {
    return Parte1;
  }


  public void startLevelCommand() {
    if (LevelEscolhido == 0 || ComandoLevelIniciou)
      return;

    switch (LevelEscolhido) {
      case 1:
        new L1(Elevator, armSubsystem, Swerve, this, Controller).schedule();
        break;

      case 2:
        new L2(Elevator, armSubsystem, Swerve, this, Controller).schedule();
        break;

      case 3:
        new L3(Elevator, armSubsystem, Swerve, this, Controller).schedule();
        break;

      case 4:
        new L4(Elevator, armSubsystem, Swerve, this, Controller).schedule();
        break;

    }

    ComandoLevelIniciou = true;
  }

  // public void DefaultLevel(){
  // new DefaultLevel(Elevator, armSubsystem, Swerve, this).schedule();
  // }

  public void periodic() {

  }
}
