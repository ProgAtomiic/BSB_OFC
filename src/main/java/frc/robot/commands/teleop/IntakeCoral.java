// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  /** Creates a new IntakeCoral. */
  ElevatorSubsystem elevatorSubsystem;
  VisionSubsystem Vision = new VisionSubsystem();

  public IntakeCoral(ElevatorSubsystem ElevatorSubsystem) {
    this.elevatorSubsystem = ElevatorSubsystem;
    addRequirements(ElevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Vision.DistanciaDaTagArducam() == 9) {
      if (ElevatorSubsystem.GetPosicaoElevador() == 20 && ElevatorSubsystem.TemCoral()) {
        ElevatorSubsystem.DescerLimite();
      } else {
        ElevatorSubsystem.PIDNoFF(20); // COLOCAR LED PARA INDICAR PRO HUMAN PLAYER
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ElevatorSubsystem.DesligarElevador();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ElevatorSubsystem.Descido() ? true : false;
  }
}
