package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_SLIDE_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.INTAKE_WRIST_ANGLE;
import static org.firstinspires.ftc.teamcode.Constants.OUTTAKE_WRIST_ANGLE;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class OuttakeState extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;
    private boolean closeLoop = false;
    private double slidePos = OUTTAKE_SLIDE_POSITION;

    public OuttakeState(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, GamepadEx gamepad1, GamepadEx gamepad2){
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        addRequirements(armSubsystem, clawSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.moveArmToMax();
        armSubsystem.retractArmToMin();
        clawSubsystem.closeClaw();
        clawSubsystem.clawBack();
        closeLoop = false;
    }

    @Override
    public void execute(){
        closeLoop = false;
        if (armSubsystem.getArmPosition() > OUTTAKE_ARM_POSITION + EXTENSION_ERROR) {
            armSubsystem.stopMotors();
        }
        if (armSubsystem.getViperPosition() < slidePos && armSubsystem.getViperPosition() > slidePos - EXTENSION_ERROR) {
            armSubsystem.stopSlide();
        }
        if (gamepad1.getButton(GamepadKeys.Button.DPAD_UP)) {
            armSubsystem.extendArmToMax();
            this.slidePos = MAX_EXTEND_POS;
        }
        if (gamepad1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            armSubsystem.retractArmToMin();
            this.slidePos = MIN_EXTEND_POS;
        }
        if (gamepad1.getButton(GamepadKeys.Button.X)) {
            clawSubsystem.openClaw();
            armSubsystem.retractArmToMin();
            this.slidePos = MIN_EXTEND_POS;

        }
        if (!gamepad1.getButton(GamepadKeys.Button.X) && clawSubsystem.clawIsOpen() && armSubsystem.getViperPosition() < MIN_EXTEND_POS + EXTENSION_ERROR && armSubsystem.getViperPosition() > MIN_EXTEND_POS - EXTENSION_ERROR) {
            closeLoop = true;
        }
        if (gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            clawSubsystem.clawBack();
        }
        if (gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            clawSubsystem.clawIntake();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        clawSubsystem.openClaw();
    }

    @Override
    public boolean isFinished() {
        return closeLoop;
    }
}
