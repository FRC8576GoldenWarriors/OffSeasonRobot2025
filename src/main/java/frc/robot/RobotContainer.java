package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Constants;
import frc.robot.Commands.ElevatorController;
import frc.robot.Commands.SwerveDrive;

public class RobotContainer {

  //   public static final Drivetrain m_drivetrain = Drivetrain.getInstance();

  public static final CommandXboxController driverController =
      new CommandXboxController(Constants.ControllerConstants.driverControllerPort);
  public static final CommandXboxController operatorController =
      new CommandXboxController(Constants.ControllerConstants.operatorControllerPort);

  public final JoystickButton resetHeading_Start =
      new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);

  public final SendableChooser<Command> autoChooser;

  public static Drivetrain m_drivetrain;
  public static Elevator m_elevator;

  public RobotContainer() {

    m_drivetrain = Drivetrain.getInstance();
    m_drivetrain.setDefaultCommand(new SwerveDrive());

    m_elevator = new Elevator();

    // Add all the choices of Autonomous modes to the Smart Dashboar
    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    resetHeading_Start.onTrue(new InstantCommand(m_drivetrain::zeroHeading, m_drivetrain));

    //move to l1
    driverController
        .povRight()
        .onTrue(
            new ElevatorController(
                m_elevator,
                Constants.ElevatorConstants.L1Position, 
                Constants.ElevatorConstants.ELEVATOR_SPEED));

    //move to l2
    driverController
    .povDown()
    .onTrue(
        new ElevatorController(
            m_elevator,
            Constants.ElevatorConstants.L2Position, 
            Constants.ElevatorConstants.ELEVATOR_SPEED));

    //move to l3
    driverController
    .povLeft()
    .onTrue(
        new ElevatorController(
            m_elevator,
            Constants.ElevatorConstants.L3Position, 
            Constants.ElevatorConstants.ELEVATOR_SPEED));

    //move to l4
    driverController
    .povUp()
    .onTrue(
        new ElevatorController(
            m_elevator,
            Constants.ElevatorConstants.L4Position, 
            Constants.ElevatorConstants.ELEVATOR_SPEED));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {

  }
}
