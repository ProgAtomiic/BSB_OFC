package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.auto.Alinhamento1;
import frc.robot.commands.auto.Alinhamento2;
import frc.robot.commands.auto.L4_1;
import frc.robot.commands.auto.L4_2;
import frc.robot.commands.teleop.Alga.AlinhamentoAlga;
import frc.robot.commands.teleop.Alga.BallIntake;
import frc.robot.commands.teleop.Alga.BallSetPoint;
import frc.robot.commands.teleop.Alga.BallShooter;
//CORAL
import frc.robot.commands.teleop.Reef.AlinhamentoReef;
import frc.robot.commands.teleop.Reef.IntakeCoral;
import frc.robot.commands.teleop.Reef.TiraBolaBaixa;
import frc.robot.commands.teleop.Reef.TiraBolaAlta;
import frc.robot.commands.teleop.Reef.L1;
import frc.robot.commands.teleop.Reef.ResetLevel;
//ALGA
import frc.robot.subsystems.AlgaSubsystem;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.commands.teleop.BallTakeOut;
import frc.robot.subsystems.ElevatorSubsystem;
//SWERVE
import frc.robot.subsystems.SwerveSubsystem;
//CAMERAS
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

        // region Subsistemas+comandos
        private final static SwerveSubsystem drivebase = new SwerveSubsystem();
        private static final XboxController Controle_0 = new XboxController(OperatorConstants.ControlePrincipal);
        private static final XboxController Controle_2 = new XboxController(OperatorConstants.ControleSecundario);

        Trigger RightStick = new JoystickButton(Controle_0, XboxController.Button.kRightStick.value);

        public static final AlgaSubsystem m_alga_subsystem = new AlgaSubsystem();
        public static final BallIntake m_ball_intake = new BallIntake(m_alga_subsystem, Controle_0);
        public static final BallShooter m_ball_shooter = new BallShooter(m_alga_subsystem);
        public static final BallSetPoint m_ball_setpoint = new BallSetPoint(m_alga_subsystem);

        public static final ArmSubsystem ARM_SUBSYSTEM = new ArmSubsystem();

        public static final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
        public static final LevelSet m_LevelSet = new LevelSet(m_ElevatorSubsystem, drivebase, ARM_SUBSYSTEM,
                        Controle_2);

        public static final VisionSubsystem Visao = new VisionSubsystem();
        public static final IntakeCoral IntakeCoral = new IntakeCoral(m_ElevatorSubsystem, ARM_SUBSYSTEM);
        public static final AlinhamentoAlga m_alinhamento_alga = new AlinhamentoAlga(Controle_0, Controle_2, drivebase,
                        m_alga_subsystem);
        public static final AlinhamentoReef AlinhamentoReef = new AlinhamentoReef(ARM_SUBSYSTEM, m_ElevatorSubsystem,
                        Controle_0, Controle_2, drivebase, m_LevelSet);
        public static final TiraBolaBaixa TiraBolaBaixa = new TiraBolaBaixa(m_alga_subsystem, m_ElevatorSubsystem,
                        ARM_SUBSYSTEM);
        public static final TiraBolaAlta TiraBolaAlta = new TiraBolaAlta(m_alga_subsystem, m_ElevatorSubsystem,
                        ARM_SUBSYSTEM);
        public static final L1 L1 = new L1(m_ElevatorSubsystem, ARM_SUBSYSTEM, drivebase, m_LevelSet, Controle_0);

        private final Map<JoystickButton, Integer> MapBotaoLevel = new HashMap<>();

        public static final AlinhamentoAlga AlinhamentoAlga = new AlinhamentoAlga(Controle_0, Controle_2, drivebase,
                        m_alga_subsystem);

        // endregion
        public RobotContainer() {

                NamedCommands.registerCommand("Alinhamento_1", new Alinhamento1(ARM_SUBSYSTEM, m_ElevatorSubsystem, Controle_0, Controle_2, drivebase, m_LevelSet));
                NamedCommands.registerCommand("Alinhamento_2", new Alinhamento2(ARM_SUBSYSTEM, m_ElevatorSubsystem, Controle_0, Controle_2, drivebase, m_LevelSet));

                NamedCommands.registerCommand("L4_1", new L4_1(m_ElevatorSubsystem, ARM_SUBSYSTEM, drivebase, m_LevelSet, Controle_0));
                NamedCommands.registerCommand("L4_2", new L4_2(m_ElevatorSubsystem, ARM_SUBSYSTEM, drivebase, m_LevelSet, Controle_0));

                NamedCommands.registerCommand("ResetLevel", new ResetLevel(m_ElevatorSubsystem, ARM_SUBSYSTEM));


                NamedCommands.registerCommand("L1", new L1(m_ElevatorSubsystem, ARM_SUBSYSTEM, drivebase, m_LevelSet, Controle_0));


                configureBindings();
        }

        // region Swerve
        SwerveInputStream DriveVelAngular = SwerveInputStream.of(
                drivebase.getSwerveDrive(),
                () -> Controle_0.getLeftY() * -1,
                () -> Controle_0.getLeftX() * -1)
                .withControllerRotationAxis(Controle_0::getRightX)
                .deadband(SwerveConstants.ZonaMorta)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        //Baixa velocidade      
        SwerveInputStream DriveVelAngularLow = SwerveInputStream.of(
                drivebase.getSwerveDrive(),
                () -> Controle_0.getLeftY() * -1,
                () -> Controle_0.getLeftX() * -1)
                .withControllerRotationAxis(Controle_0::getRightX)
                .deadband(SwerveConstants.ZonaMorta)
                .scaleTranslation(0.05)
                .allianceRelativeControl(true);

        SwerveInputStream driveDirectAngle = DriveVelAngular.copy().withControllerHeadingAxis(
                Controle_0::getRightX,
                Controle_0::getRightY)
                .headingWhile(true);

        // SwerveInputStream DriveLow = DriveVelAngularLow.copy().withControllerHeadingAxis(
        // Controle_0::getRightX,
        // Controle_0::getRightY)
        // .headingWhile(true);

        SwerveInputStream DriveLow = DriveVelAngularLow.copy().withControllerRotationAxis(
                Controle_0::getRightX)//TODO: TALVEZ MUDAR PRA LEFTX
                .headingWhile(true);


                
        // Command driveFieldOrientedDirectAngle =
        // drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(DriveVelAngular);

        Command driveRobotSlow = drivebase.driveFieldOriented(DriveLow);
        // Command driveRobotOriented = drivebase.driveRobotOriented(DriveVelAngular);

        private void configureBindings() {
                // SWERVE
                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
                // drivebase.setDefaultCommand(driveRobotOriented);

                new JoystickButton(Controle_0, XboxController.Button.kX.value).toggleOnTrue(driveRobotSlow);
                new JoystickButton(Controle_2, XboxController.Button.kY.value).onTrue(L1);


                // ALGA SUBSYSTEM
                new Trigger(() -> Controle_0.getLeftTriggerAxis() > 0.25).whileTrue(m_ball_intake);
                new Trigger(() -> Controle_0.getRightTriggerAxis() > 0.25).whileTrue(m_ball_shooter);
                new Trigger(() -> Controle_0.getLeftTriggerAxis() < 0.25 && Controle_0.getRightTriggerAxis() < 0.25).whileTrue(m_ball_setpoint);


                // ELEVADOR SUBSYSTEM
                new Trigger(() -> Controle_0.getRightStickButton()).toggleOnTrue(IntakeCoral);
                new Trigger(() -> Controle_0.getYButton()).toggleOnTrue(TiraBolaAlta);
                new Trigger(() -> Controle_0.getAButton()).toggleOnTrue(TiraBolaBaixa);


                // Esocolha lado
                new JoystickButton(Controle_0, XboxController.Button.kLeftBumper.value)
                                .onTrue(new InstantCommand(() -> m_LevelSet.selectSide("Esquerda")));

                new JoystickButton(Controle_0, XboxController.Button.kRightBumper.value)
                                .onTrue(new InstantCommand(() -> m_LevelSet.selectSide("Direita")));

                

                MapBotaoLevel.put(new JoystickButton(Controle_2, XboxController.Button.kB.value), 3);
                MapBotaoLevel.put(new JoystickButton(Controle_2, XboxController.Button.kA.value), 4);

                for (Map.Entry<JoystickButton, Integer> entry : MapBotaoLevel.entrySet()) {
                        JoystickButton Botao = entry.getKey();
                        int Level = entry.getValue();

                        Botao.onTrue(new InstantCommand(() -> {
                                m_LevelSet.selectLevel(Level);
                        }));
                }

                new Trigger(() -> m_LevelSet.PodeAlinhar()).onTrue(new InstantCommand(this::ComecaAlinhamento));
                new Trigger(() -> Controle_0.getStartButton()).onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

                new JoystickButton(Controle_2, XboxController.Button.kStart.value).onTrue(Commands.runOnce(() -> drivebase.resetGyro()));
        }

        private void ComecaAlinhamento() {
                new AlinhamentoReef(ARM_SUBSYSTEM, m_ElevatorSubsystem, Controle_0, Controle_2, drivebase, m_LevelSet)
                                .schedule();
        }

        public Command getAutonomousCommand() {

                // return new PathPlannerAuto("AutonomoOFC1");
                return new PathPlannerAuto("AutonomoL4");
                // return new PathPlannerAuto("AUTOTRAS");

                // return new AUTONOMO(ARM_SUBSYSTEM, m_ElevatorSubsystem, Controle_0,
                // Controle_2, drivebase, m_LevelSet);

        }

}
