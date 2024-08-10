package frc.robot;

<<<<<<< HEAD
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  public static double driveSpeed;
  public static double turnSpeed;
  public static double rot;

  public Drivetrain drivetrain = new Drivetrain();

  public static final Joystick driveControl = new Joystick(0);
  public static final Joystick turnControl = new Joystick(1);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
=======
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();

  private static final XboxController driveControl = new XboxController(0);
  private static final XboxController turnControl = new XboxController(1); 

>>>>>>> origin/master
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

<<<<<<< HEAD
 
   
=======
>>>>>>> origin/master
  private void configureBindings() {

  }
}
