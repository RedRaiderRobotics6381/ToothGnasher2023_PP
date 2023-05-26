package frc.robot.commands.Arm.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Secondary.ArmSubsystem;

public class ArmWristCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;
    boolean clockwise = true;

    // This file rotates the wrist motor. It has to move in two different directions, so that it doesn't break the physical motor.

    /**
     * Turns the wrist
     * @param armSubsystem *Subsystem* ArmSubsystem
     * @return *Void* Sets the wrist motors.
     */
    public ArmWristCmd(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Sets the motor direction based on whether it needs to go clockwise or counterclockwise
        if(clockwise == true){
            armSubsystem.wristRotateMotor.set(0.9 * ((Math.abs(armSubsystem.wristRotateEncoder.getPosition()-90)+20)/180));
        } else{
            armSubsystem.wristRotateMotor.set(-0.9 * ((Math.abs(armSubsystem.wristRotateEncoder.getPosition()-270)+50)/180));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // This switches it from whether it needs to be clockwise or counterclockwise by the next turn
        armSubsystem.wristRotateMotor.set(0);
        if(clockwise == true){
            clockwise = false;
        } else {
            clockwise = true;
        }
    }

    @Override
    public boolean isFinished() {
        // Determins if it is finished based on whether it is clockwise or not
        if(clockwise == true){
            if(armSubsystem.wristRotateEncoder.getPosition() > 90){
                return false;
            } else{
                return true;
            }
        } else {
            if(armSubsystem.wristRotateEncoder.getPosition() >= 250){
                return true;
            } else{
                return false;
            }
        }
    }
}