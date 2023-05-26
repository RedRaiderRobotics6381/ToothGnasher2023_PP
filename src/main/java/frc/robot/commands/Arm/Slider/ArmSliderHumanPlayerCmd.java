package frc.robot.commands.Arm.Slider;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Secondary.ArmSubsystem;

public class ArmSliderHumanPlayerCmd extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmSliderHumanPlayerCmd(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // up
        if(armSubsystem.sliderEncoder.getPosition() > -Constants.ArmConstants.gArmSliderHumanPlayer + Constants.ArmConstants.gArmOffset){
            armSubsystem.leftArmSlider.set(Constants.ArmConstants.gSliderSpeed);
            armSubsystem.rightArmSlider.set(-Constants.ArmConstants.gSliderSpeed);
        }
        // down
        else if(armSubsystem.sliderEncoder.getPosition() < -Constants.ArmConstants.gArmSliderHumanPlayer - Constants.ArmConstants.gArmOffset){
            armSubsystem.leftArmSlider.set(-Constants.ArmConstants.gSliderDown);
            armSubsystem.rightArmSlider.set(Constants.ArmConstants.gSliderDown);
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.leftArmSlider.set(0);
        armSubsystem.rightArmSlider.set(0);
    }

    @Override
    public boolean isFinished() {
        System.out.println(armSubsystem.sliderEncoder.getPosition() + " Human Player");
        if(armSubsystem.sliderEncoder.getPosition() < -Constants.ArmConstants.gArmSliderHumanPlayer + Constants.ArmConstants.gArmOffset && armSubsystem.sliderEncoder.getPosition() > -Constants.ArmConstants.gArmSliderHumanPlayer - Constants.ArmConstants.gArmOffset){
            return true;
        } else{
            return false;
        }
    }
}