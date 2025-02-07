// package frc.robot.commands.teleop;

// import java.util.ArrayList;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.AlgaSubsystem;

// public class BallIntake extends Command {
//   private final AlgaSubsystem intakealga;
//   private final XboxController Controle_0;
//   // boolean finished = false;

//   static ArrayList<Integer> parametros = new ArrayList<>();

//   public BallIntake(AlgaSubsystem Intakealga, XboxController Controle_0) {
//     this.intakealga = Intakealga;
//     this.Controle_0 = Controle_0;
//     addRequirements(intakealga);
//   }

//   // Called when the command is initially scheduled.
//   // Called every time the scheduler runs while the command is scheduled.

//   @Override
//   public void execute() {
//     AlgaSubsystem.linhaset(360, 0.2);
//     AlgaSubsystem.ligaintake(0.2);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     AlgaSubsystem.desligaintake();
//     AlgaSubsystem.desligalinha();
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }

// }
