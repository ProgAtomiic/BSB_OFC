
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlinhamentoConstants;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Add your docs here. */
public class SwerveSubsystem extends SubsystemBase {

  public static final int DistanciaEncoder = 0;

  // region VARIÁVEIS
  private final VisionSubsystem Vision = new VisionSubsystem();

  public static double VelocidadeSwerve = 1;

  // PUBLICA A ODOMETRIA NO ADVANTAGE SCOPE
  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct).publish();

  public int SwitchOdometria = 0;

  //
  // public Vector<N3> visionMeasurementsStdDevs = VecBuilder.fill(0.9, 0.9,
  // Units.degreesToRadians(10));
  public Vector<N3> visionMeasurementsStdDevs = VecBuilder.fill(2, 2, Units.degreesToRadians(20));
  public EstimatedRobotPose chosenPose;

  File Diretorio = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;

  // endregion

  public SwerveSubsystem() {
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(Diretorio).createSwerveDrive(
          SwerveConstants.VelMax,
          new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // TODO: AZUL
    // swerveDrive.resetOdometry(new Pose2d(
    // Units.inchesToMeters(325.68),
    // Units.inchesToMeters(241.64),
    // swerveDrive.getYaw().plus(Rotation2d.fromDegrees(180)) // Offset by 180°
    // ));

    // TODO: VERMELHO
    // swerveDrive.resetOdometry(new Pose2d(
    // Units.inchesToMeters(325.68),
    // Units.inchesToMeters(241.64),
    // swerveDrive.getYaw() // Offset by 180°
    // ));

    setupPathPlanner();
    swerveDrive.setHeadingCorrection(true);

  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void resetGyro() {
    swerveDrive.zeroGyro();
  }

  public VisionSubsystem getVision() {
    return Vision;
  }

  // region METODOS SWERVE
  public void driveFieldOriented(ChassisSpeeds Velocidade) {
    swerveDrive.driveFieldOriented(Velocidade);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> Velocidade) {
    return run(() -> {
      swerveDrive.driveFieldOriented(Velocidade.get());
    });
  }

  public Command driveRobotOriented(Supplier<ChassisSpeeds> Velocidade) {
    return run(() -> {
      swerveDrive.drive(Velocidade.get());
    });
  }

  // endregion

  // region PATHPLANNER
  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      final boolean enableFeedforward = true;
      AutoBuilder.configure(
          swerveDrive::getPose,
          swerveDrive::resetOdometry,
          swerveDrive::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(0.005, 0.0, 0.4),
              new PIDConstants(0.005, 0, 0.6)

          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },

          this);

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public Command getAutonomousCommand(String AutoName) {
    return new PathPlannerAuto(AutoName);
  }

  public Command getPathCommand(String PathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);
      PathConstraints constraints = new PathConstraints(
          3.0, 4.0,
          Units.degreesToRadians(540), Units.degreesToRadians(720));

      return AutoBuilder.pathfindThenFollowPath(path, constraints);
    } catch (IOException | ParseException e) {
      e.printStackTrace();
      return Commands.none();
    }
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public double DistanciaEncoder() {
    return swerveDrive.getModulePositions()[0].distanceMeters;
  }

  // endregion
  public void periodic() {

    // var currentCommand = getCurrentCommand();
    // if (currentCommand != null) {
    // System.out.println("[ElevatorSubsystem] Running Command: " +
    // currentCommand.getName());
    // }

    // Optional<EstimatedRobotPose> estimatedPoseOpt =
    // Vision.getEstimatedPoseLime();
    // if (estimatedPoseOpt.isPresent()) {
    // if (estimatedPoseOpt.isPresent()) {
    // //
    // System.out.println(estimatedPoseOpt.get().estimatedPose.toPose2d().getRotation().getDegrees());
    // // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // // estimatedPoseOpt.get().estimatedPose.toPose2d().getRotation(),
    // // estimatedPoseOpt.get().timestampSeconds,
    // // visionMeasurementsStdDevs);
    // }
    // }

    // Optional<EstimatedRobotPose> estimatedPoseOpt =
    // Vision.getEstimatedPoseLime();

    swerveDrive.setChassisDiscretization(true, true, 0.02);
    swerveDrive.setHeadingCorrection(false);

    // System.out.println(swerveDrive.getOdometryHeading());
    // // Variáveis globais para estabilidade da escolha
    // private static final double AMBIGUITY_THRESHOLD = 0.15; // Ajustável conforme
    // necessário
    // private String lastChosenCamera = "limelight"; // Inicialmente priorizando a
    // Limelight
    // private int stableChoiceCounter = 0;
    // private static final int STABILITY_COUNT = 5; // Precisa ser melhor por 5
    // ciclos seguidos

    // Optional<EstimatedRobotPose> aruPose = getVision().getEstimatedPoseArducam();
    // Optional<EstimatedRobotPose> limePose = getVision().getEstimatedPoseLime();

    // if (aruPose.isPresent() && limePose.isPresent()) {
    // PhotonTrackedTarget aruTarget = aruPose.get().estimatedPose.getBestTarget();
    // PhotonTrackedTarget limeTarget =
    // limePose.get().estimatedPose.getBestTarget();

    // int aruTagID = aruTarget.getFiducialId();
    // int limeTagID = limeTarget.getFiducialId();

    // // Se as duas câmeras veem a mesma tag
    // if (aruTagID == limeTagID) {
    // double aruAmbiguity = aruTarget.getPoseAmbiguity();
    // double limeAmbiguity = limeTarget.getPoseAmbiguity();

    // // Filtro para evitar instabilidades:
    // // Se uma câmera for melhor por STABILITY_COUNT ciclos consecutivos, trocamos
    // // para ela
    // if (aruAmbiguity < limeAmbiguity - AMBIGUITY_THRESHOLD) {
    // if (lastChosenCamera.equals("arducam")) {
    // stableChoiceCounter = 0; // Já estamos na melhor câmera
    // } else {
    // stableChoiceCounter++;
    // if (stableChoiceCounter >= STABILITY_COUNT) {
    // lastChosenCamera = "arducam"; // Troca de câmera somente após estabilidade
    // stableChoiceCounter = 0;
    // }
    // }
    // } else if (limeAmbiguity < aruAmbiguity - AMBIGUITY_THRESHOLD) {
    // if (lastChosenCamera.equals("limelight")) {
    // stableChoiceCounter = 0;
    // } else {
    // stableChoiceCounter++;
    // if (stableChoiceCounter >= STABILITY_COUNT) {
    // lastChosenCamera = "limelight";
    // stableChoiceCounter = 0;
    // }
    // }
    // }

    // // Escolher a câmera final com base na escolha estável
    // EstimatedRobotPose chosenPose = lastChosenCamera.equals("arducam") ?
    // aruPose.get() : limePose.get();

    // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // chosenPose.estimatedPose.toPose2d(),
    // chosenPose.timestampSeconds,
    // visionMeasurementsStdDevs);

    // } else {
    // // Tags diferentes - escolher a câmera principal (ex: Limelight)
    // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // limePose.get().estimatedPose.toPose2d(),
    // limePose.get().timestampSeconds,
    // visionMeasurementsStdDevs);
    // }

    // } else if (aruPose.isPresent()) {
    // // Apenas Arducam detectou uma tag
    // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // aruPose.get().estimatedPose.toPose2d(),
    // aruPose.get().timestampSeconds,
    // visionMeasurementsStdDevs);

    // } else if (limePose.isPresent()) {
    // // Apenas Limelight detectou uma tag
    // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // limePose.get().estimatedPose.toPose2d(),
    // limePose.get().timestampSeconds,
    // visionMeasurementsStdDevs);
    // }

    publisher.set(swerveDrive.getPose());

  }

}
