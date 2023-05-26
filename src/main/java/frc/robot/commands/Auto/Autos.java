package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.HashMap;

public final class Autos {
  private static HashMap<String, Command> eventMap = new HashMap<>();
  public static SwerveAutoBuilder autoBuilder;
  // public static ArmSubsystem armSubsystem;
  public static final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static void init() {
    autoBuilder = new SwerveAutoBuilder(
        RobotContainer.swerveSubsystem::getPose,
        RobotContainer.swerveSubsystem::resetOdometry,
        DriveConstants.kDriveKinematics,
        new PIDConstants(0,0.0, 0.0),
        new PIDConstants(.8, 0.35, 0.00),
        RobotContainer.swerveSubsystem::setModuleStates,
        eventMap,
        true,
        RobotContainer.swerveSubsystem);
    // armSubsystem = new ArmSubsystem();

    autoChooser.setDefaultOption("None", none());
    autoChooser.addOption("Simple Test Path", new Auto_SimplePath(autoBuilder, RobotContainer.armSubsystem));
    autoChooser.addOption("Auto_1A", new Auto_1A(autoBuilder));
    autoChooser.addOption("Auto_2A", new Auto_2A(autoBuilder));
    autoChooser.addOption("Auto_3A", new Auto_3A(autoBuilder));
    autoChooser.addOption("Auto_4B", new Auto_4B(autoBuilder));
    SmartDashboard.putData("Autonomous Mode", autoChooser);
    Shuffleboard.getTab("Autonomous").add(autoChooser);
  }
  
  public static CommandBase none() {
    return Commands.none();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
