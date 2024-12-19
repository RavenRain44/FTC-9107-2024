package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExampleSubsystem;

public class DefaultState extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final ClawSubsystem clawSubsystem;

    public DefaultState(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem){
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;
        addRequirements(armSubsystem, clawSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.moveArmToPosition(DEFAULT_ARM_POSITION);
        armSubsystem.retractArmToMin();
        clawSubsystem.closeClaw();
        clawSubsystem.clawBack();
    }

    @Override
    public void execute(){
        if (armSubsystem.getArmPosition() < DEFAULT_ARM_POSITION + EXTENSION_ERROR && armSubsystem.getArmPosition() > DEFAULT_ARM_POSITION - EXTENSION_ERROR) {
            armSubsystem.stopMotors();
        }
        if (armSubsystem.getViperPosition() < DEFAULT_SLIDE_POSITION + EXTENSION_ERROR) {
            armSubsystem.stopSlide();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
