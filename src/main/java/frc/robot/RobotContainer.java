package frc.robot;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();

  private static final XboxController driveControl = new XboxController(0);
  private static final XboxController turnControl = new XboxController(1); 

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(
      new RunCommand(() -> drivetrain.drive(
        driveControl.getLeftY(), 
        driveControl.getLeftX(), 
        turnControl.getRightX(),
        true // field centric vs robot centric
      ), drivetrain)
    );
  }

  private void configureBindings() {
    // Additional button bindings can be added here if needed
  }
}
