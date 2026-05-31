package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.AutoRoutines;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.MapleSimSwerve;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.RobotIdentity;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private SwerveDriveSimulation swerveDriveSimulation = null;

  public static boolean isRed() {
    var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    return alliance.isPresent()
        && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
  }

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().FrontLeft),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().FrontRight),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().BackLeft),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().BackRight),
                swerveDriveSimulation);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        break;

      case SIM:
        swerveDriveSimulation =
            MapleSimSwerve.createSimulationDrive(RobotIdentity.getTunerConstants());
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(swerveDriveSimulation.getModules()[0]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[1]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[2]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[3]),
                swerveDriveSimulation);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, swerveDriveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name,
                    robotToCamera1,
                    swerveDriveSimulation::getSimulatedDriveTrainPose));
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                swerveDriveSimulation);
        vision = new Vision(drive::addVisionMeasurement, drive::getPose, new VisionIO() {});
        break;
    }

    // Set up auto routines
    LoggedDashboardChooser<Command> tempChooser;
    try {
      AutoRoutines autoRoutines = new AutoRoutines(drive);
      tempChooser = new LoggedDashboardChooser<>("Auto Choices", autoRoutines.buildAutoChooser());
      tempChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      tempChooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      tempChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      tempChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      tempChooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      tempChooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    } catch (Exception e) {
      e.printStackTrace();
      Alert alert = new Alert("auto failed to load", AlertType.kError);
      alert.set(true);
      tempChooser = new LoggedDashboardChooser<>("Auto Choices");
    }
    autoChooser = tempChooser;

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new edu.wpi.first.math.geometry.Pose2d(
                                drive.getPose().getTranslation(),
                                isRed()
                                    ? edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)
                                    : edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180))),
                    drive)
                .ignoringDisable(true));

    // X-lock wheels
    controller.x().whileTrue(Commands.run(drive::stopWithX, drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateSimulation() {
    if (swerveDriveSimulation != null) {
      Logger.recordOutput(
          "Odometry/SimulatedPose", swerveDriveSimulation.getSimulatedDriveTrainPose());
    }
  }
}
