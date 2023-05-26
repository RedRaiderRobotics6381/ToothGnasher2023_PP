package frc.robot.commands.Auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class Auto_1A extends SequentialCommandGroup {

    public Auto_1A(SwerveAutoBuilder autoBuilder) {
        List<PathPlannerTrajectory> pathGroup =
            PathPlanner.loadPathGroup("Auto_1A", new PathConstraints(
                0.8 * Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                1.0 * Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        
        addCommands(
            Commands.sequence(
                    autoBuilder.resetPose(pathGroup.get(0)),
                    autoBuilder.followPathWithEvents(pathGroup.get(0))));
        }
}