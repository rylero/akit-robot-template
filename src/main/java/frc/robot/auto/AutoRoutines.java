package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.LinkedHashMap;
import java.util.Map;

public class AutoRoutines {
  private final Drive drive;

  public AutoRoutines(Drive drive) {
    this.drive = drive;
  }

  public Map<String, Command> buildAutoChooser() {
    Map<String, Command> autos = new LinkedHashMap<>();
    autos.put("Do Nothing", Commands.none());
    // TODO: Add your auto routines here using AutoBuilder.buildAuto("AutoName")
    return autos;
  }
}
