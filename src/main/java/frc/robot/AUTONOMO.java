// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.commands.teleop.Reef.AutoDireita;
// import frc.robot.commands.teleop.Reef.L4;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class AUTONOMO extends Command {
//   int AUTO;
//   private final SwerveSubsystem SwerveSubsystem;
//   private final ArmSubsystem Arm;
//   private final ElevatorSubsystem Elevator;
//   private final XboxController Controle_0;
//   private final XboxController Controle_2;
//   double TempoAuto;

//   // private final int Level;

//   // private final String Lado;
//   private final LevelSet LevelSet;
//   private final VisionSubsystem Vision = new VisionSubsystem();

//   /** Creates a new AUTONOMO. */
//   public AUTONOMO(ArmSubsystem Arm, ElevatorSubsystem Elevator, XboxController Controle_0,
//       XboxController Controle_2, SwerveSubsystem SwerveSubsystem, LevelSet LevelSet) {
//     this.Elevator = Elevator;
//     this.Arm = Arm;
//     this.Controle_0 = Controle_0;
//     this.Controle_2 = Controle_2;
//     // this.Level = Level;
//     this.SwerveSubsystem = SwerveSubsystem;
//     // this.Lado = Lado;
//     this.LevelSet = LevelSet;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     switch (AUTO) {

//       case 1:
//         PIDController PID = new PIDController(2.7, 0.0, 0);

//         if (frc.robot.subsystems.SwerveSubsystem.DistanciaEncoder < 1) {
//           SwerveSubsystem.getSwerveDrive()
//               .drive(new ChassisSpeeds(PID.calculate(frc.robot.subsystems.SwerveSubsystem.DistanciaEncoder, 1), 0, 0));

//         } else {
//           AUTO = 2;
//         }
//         break;

//       case 2:
//         PIDController PID = new PIDController(2.7, 0.0, 0);

//         if (frc.robot.subsystems.SwerveSubsystem.DistanciaEncoder < 2) {
//           SwerveSubsystem.getSwerveDrive()
//               .drive(new ChassisSpeeds(PID.calculate(frc.robot.subsystems.SwerveSubsystem.DistanciaEncoder, 2), 0, 0));

//         } else {
//           TempoAuto = Timer.getFPGATimestamp();
//           AUTO = 3;
//         }
//         break;

//       case 3:
//         PIDController PID = new PIDController(2.7, 0.0, 0);

//         if (Timer.getFPGATimestamp() - TempoAuto < 1) {
//           SwerveSubsystem.getSwerveDrive().drive(new ChassisSpeeds(1, 0, 0));

//         } else {
//           AUTO = 4;
//         }
//         break;

//       case 4:

//         new AutoDireita(Arm, Elevator, Controle_0,Controle_2, SwerveSubsystem, LevelSet).schedule();
//         AUTO = 5; //TODO NAO VAI FUNCIONAR, PRECISA DE UMA CONDIÇÃO PARA O COMANDO TERMINAR
//         break;

//       case 5:
//         PIDController PID = new PIDController(2.7, 0.0, 0);

//         if (SwerveSubsystem.DistanciaEncoder < 2) {
//           SwerveSubsystem.getSwerveDrive()
//               .drive(new ChassisSpeeds(0, PID.calculate(SwerveSubsystem.DistanciaEncoder, 2), 0));
//         }

//         break;

//     }

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
