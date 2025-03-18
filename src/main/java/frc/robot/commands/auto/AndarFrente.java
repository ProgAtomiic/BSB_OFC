// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AndarFrente extends Command {
  private final SwerveSubsystem swerveSubsystem;
  double Tempo;
  boolean Terminado;
  /** Creates a new AndarFrente. */
  public AndarFrente(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Tempo = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Timer.getFPGATimestamp() - Tempo < 0.5){
      swerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(-0.6, 0, 0));
    }else{
      swerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
      Terminado = true;

    }
  


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Terminado = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Terminado;
  }
}
