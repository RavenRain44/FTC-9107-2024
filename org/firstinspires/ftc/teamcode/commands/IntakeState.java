package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class IntakeState extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final GamepadEx gamepad1;
    private boolean closeLoop = false;

    public IntakeState(ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, GamepadEx gamepad1){
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.gamepad1 = gamepad1;
        addRequirements(armSubsystem, clawSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.armIntakeMode();
        armSubsystem.prepareIntake();
        clawSubsystem.openClaw();
        clawSubsystem.clawIntake();
        closeLoop = false;
    }

    @Override
    public void execute(){
        closeLoop = false;
        double expectedArmPosition = armSubsystem.angleToArmUnits(Math.toDegrees(Math.asin(HEIGHT / armSubsystem.slideUnitsToIn(armSubsystem.getViperPosition()))));
        clawSubsystem.setAppWristAngleZero(armSubsystem);
        if ((armSubsystem.getArmPosition() > (expectedArmPosition * 0.995)) && (armSubsystem.getArmPosition() < (expectedArmPosition * 1.005))) {
            armSubsystem.stopMotors();
        } else {
            armSubsystem.setArmToDistance(expectedArmPosition);
        }
        if (gamepad1.getButton(GamepadKeys.Button.DPAD_UP)) {
            armSubsystem.setSlidePower(MAX_VIPER_POWER * 2 / 3);
            armSubsystem.armRawPower(0.25);
        } else if (gamepad1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            armSubsystem.setSlidePower(-MAX_VIPER_POWER * 2 / 3);
        } else {
            armSubsystem.stopSlide();
        }
        if (gamepad1.getButton(GamepadKeys.Button.X)) {
            clawSubsystem.closeClaw();
        }
        if (!gamepad1.getButton(GamepadKeys.Button.X) && clawSubsystem.clawIsClosed()) {
            closeLoop = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return closeLoop;
    }
}
