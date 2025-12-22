package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.Drivetrain.Drivetrain;

public class RobotContainer {
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Joystick joystick = new Joystick(0);

  public RobotContainer() {
    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> drivetrain.drive(
                -joystick.getY() * 0.5, 
                joystick.getX() * 0.5, 
                joystick.getRawAxis(4) * 0.5
            ), 
            drivetrain
        )
    );
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
