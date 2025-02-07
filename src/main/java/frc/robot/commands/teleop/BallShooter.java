// package frc.robot.commands.teleop;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.XboxController;
// import java.util.ArrayList;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.AlgaSubsystem;
// import frc.robot.subsystems.ArmSubsystem;

// public class BallShooter extends Command {
//   private final AlgaSubsystem shooterAlga;
//   private final XboxController Controle_0;
//   // boolean finished = false;

//   static ArrayList<Integer> parametros = new ArrayList<>();

//   public BallShooter(AlgaSubsystem ShooterAlga, XboxController Controle_0) {
//     this.shooterAlga = ShooterAlga;
//     this.Controle_0 = Controle_0;
//     addRequirements(shooterAlga);
//   }

//   // Called when the command is initially scheduled.
//   // Called every time the scheduler runs while the command is scheduled.

//   @Override
//   public void execute() {
//     // AQUI DEVE SER O ANGULO PARA O INTAKE VOLTAR PARA DENTRO
//     AlgaSubsystem.linhaset(-360, 0.2);
//     AlgaSubsystem.ligaintake(-0.2);
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
