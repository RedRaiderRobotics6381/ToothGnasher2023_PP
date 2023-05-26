package frc.robot.commands.Auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmCloseGrabberCmd;
import frc.robot.subsystems.ArmSubsystem;

public class Auto_SimplePath extends SequentialCommandGroup {

    public Auto_SimplePath(SwerveAutoBuilder autoBuilder, ArmSubsystem armSubsystem) {
        List<PathPlannerTrajectory> pathGroup =
            PathPlanner.loadPathGroup("SimplePathTest", new PathConstraints(
                0.8 * Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                1.0 * Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        
        addCommands(
            Commands.sequence(
                    autoBuilder.resetPose(pathGroup.get(0)),
                    autoBuilder.followPathWithEvents(pathGroup.get(0)),
                    new ArmCloseGrabberCmd(armSubsystem,()->true)));
                    
        }
    

        
}