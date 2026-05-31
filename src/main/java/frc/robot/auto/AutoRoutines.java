package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines {
  private final Drive drive;

  public AutoRoutines(Drive drive) {
    this.drive = drive;
  }

  public SendableChooser<Command> buildAutoChooser() {
    SendableChooser<Command> chooser = new SendableChooser<>();
    chooser.setDefaultOption("Do Nothing", Commands.none());
    // TODO: Add your auto routines here, e.g.:
    // chooser.addOption("My Auto", AutoBuilder.buildAuto("MyAuto"));
    return chooser;
  }
}
