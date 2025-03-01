
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;

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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Add your docs here. */
public class SwerveSubsystem extends SubsystemBase {

  // region VARIÁVEIS
  private final VisionSubsystem Vision = new VisionSubsystem();

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
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(Diretorio).createSwerveDrive(
          SwerveConstants.VelMax,
          new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive
        .resetOdometry(new Pose2d(Units.inchesToMeters(325.68), Units.inchesToMeters(241.64), swerveDrive.getYaw()));

    setupPathPlanner();
    swerveDrive.setHeadingCorrection(true);


  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
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

          this
      // Reference to this subsystem to set requirements
      );

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
 
  public double DistanciaEncoder(){
    return swerveDrive.getModulePositions()[0].distanceMeters;
  }
  // endregion
  public void periodic() {
    swerveDrive.setChassisDiscretization(true, true, 0.02);
    swerveDrive.setHeadingCorrection(false);
    System.out.println(swerveDrive.getOdometryHeading());
    // Optional<EstimatedRobotPose> aruPose = getVision().getEstimatedPoseArducam();
    // Optional<EstimatedRobotPose> limePose = getVision().getEstimatedPoseLime();
    // switch (SwitchOdometria) {
    //   case 0:
    //     if (aruPose.isPresent() && limePose.isPresent()) {
    //       double aruAmbiguity = getVision().ArducamGetLatestResult().getBestTarget().getPoseAmbiguity();
    //       double limeAmbiguity = getVision().ArducamGetLatestResult().getBestTarget().getPoseAmbiguity();
    //       if (aruAmbiguity < limeAmbiguity - 0.05) {
    //         System.out.println("Arducam A");
    //         chosenPose = aruPose.get();
    //       } else if (limeAmbiguity < aruAmbiguity - 0.05) {
    //         System.out.println("Limelight A");
    //         chosenPose = limePose.get();
    //       }
    //       swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    //           new Pose2d(chosenPose.estimatedPose.toPose2d().getMeasureX(),
    //               chosenPose.estimatedPose.toPose2d().getMeasureY(), swerveDrive.getYaw()),
    //           chosenPose.timestampSeconds,
    //           visionMeasurementsStdDevs);
    //     } else {
    //       SwitchOdometria = 1;
    //     }

    //     break;
    //   case 1:
    //     if (aruPose.isPresent() == true && limePose.isPresent() == false) {
    //       System.out.println("Arducam B");
    //       swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    //           new Pose2d(aruPose.get().estimatedPose.toPose2d().getMeasureX(),
    //               aruPose.get().estimatedPose.toPose2d().getMeasureY(), swerveDrive.getYaw()),
    //           aruPose.get().timestampSeconds,
    //           visionMeasurementsStdDevs);
    //     } else {
    //       SwitchOdometria = 2;
    //     }
    //     break;
    //   case 2:
    //     if (aruPose.isPresent() == false && limePose.isPresent() == true) {
    //       System.out.println("Limelight B");
    //       swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    //           new Pose2d(limePose.get().estimatedPose.toPose2d().getMeasureX(),
    //               limePose.get().estimatedPose.toPose2d().getMeasureY(), swerveDrive.getYaw()),
    //           limePose.get().timestampSeconds,
    //           visionMeasurementsStdDevs);
    //     } else {
    //       SwitchOdometria = 0;
    //     }
    //     break;
    //   default:
    //     break;
    // }
    // swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition();

    // Verificar se as duas câmeras detectaram tags

    // if (aruPose.isPresent() && limePose.isPresent()) {
    // // Ambas as câmeras veem tags, escolher a melhor com base na ambiguidade

    // Verificar se as duas câmeras detectaram tags
    // Optional<EstimatedRobotPose> aruPose = getVision().getEstimatedPoseArducam();
    // Optional<EstimatedRobotPose> limePose = getVision().getEstimatedPoseLime();

    // if (aruPose.isPresent() && limePose.isPresent()) {
    // // Ambas as câmeras veem tags, escolher a melhor com base na ambiguidade
    // double aruAmbiguity =
    // getVision().ArducamGetLatestResult().getBestTarget().getPoseAmbiguity();
    // double limeAmbiguity =
    // getVision().LimeGetLatestResult().getBestTarget().getPoseAmbiguity();

    // EstimatedRobotPose chosenPose;
    // if (aruAmbiguity < limeAmbiguity) {
    // // Arducam tem menor ambiguidade, usar esta
    // System.out.println("Arducam A");
    // chosenPose = aruPose.get();
    // } else {
    // // Limelight tem menor ambiguidade, usar esta
    // System.out.println("Limelight A");
    // chosenPose = limePose.get();
    // }

    // // Adicionar medição ao SwerveDrivePoseEstimator
    // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // chosenPose.estimatedPose.toPose2d(),
    // chosenPose.timestampSeconds,
    // visionMeasurementsStdDevs);

    // } else if (aruPose.isPresent()== true && limePose.isPresent() == false) {
    // System.out.println("Arducam B");

    // // Apenas a Arducam detectou uma tag
    // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // aruPose.get().estimatedPose.toPose2d(),
    // aruPose.get().timestampSeconds,
    // visionMeasurementsStdDevs);

    // } else if (limePose.isPresent() == true && aruPose.isPresent() == false) {
    // System.out.println("Limelight B");
    // // Apenas a Limelight detectou uma tag
    // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // limePose.get().estimatedPose.toPose2d(),
    // limePose.get().timestampSeconds,
    // visionMeasurementsStdDevs);
    // }

    // Caso nenhuma câmera veja um alvo, não faz nada

    // if (ArducamPoseAmbiguity < LimePoseAmbiguity) {
    // chosenPose = aruPose.get();
    // System.out.println("Arducam A");
    // } else {
    // chosenPose = limePose.get();
    // System.out.println("Limelight A");
    // }

    // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // chosenPose.estimatedPose.toPose2d(),
    // chosenPose.timestampSeconds,
    // visionMeasurementsStdDevs);
    // }
    // else if (aruPose.isPresent() == true && limePose.isPresent() == false) {
    // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // aruPose.get().estimatedPose.toPose2d(),
    // aruPose.get().timestampSeconds,
    // visionMeasurementsStdDevs);
    // System.out.println("Arducam B");

    // } else if (aruPose.isPresent() == false && limePose.isPresent() == true){
    // System.out.println("Limelight B");
    // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // limePose.get().estimatedPose.toPose2d(),
    // limePose.get().timestampSeconds,
    // visionMeasurementsStdDevs);
    // }

    // Caso nenhuma câmera veja um alvo, não faz nada
    // caso nenhuma camera veja alvo com ambiguidade menor que x, nao faz nadaa ?
    // --> precisa testar
    // //SEM JUNTAR OS DOIS

    // if (getVision().LimeGetLatesteResult().hasTargets() == true &&
    // getVision().ArducamGetLatesteResult().hasTargets() == true){
    // if (ArducamPoseAmbiguity < LimePoseAmbiguity){// nao sei se essa medida vai
    // funcionar

    // getVision().getEstimatedPoseArducam().ifPresent(estimatedRobotPose ->
    // swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(
    // new Pose2d
    // (estimatedRobotPose.estimatedPose.toPose2d().getMeasureX(),estimatedRobotPose.estimatedPose.toPose2d().getMeasureY(),
    // swerveDrive.getYaw()),
    // estimatedRobotPose.timestampSeconds,
    // visionMeasurementsStdDevs));

    publisher.set(swerveDrive.getPose());

  }

}
