package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.BasicMecanumDrive;

public class MecanumDriveCommand extends CommandBase {
    private final BasicMecanumDrive m_basicMecanumDrive;
    private final GamepadEx gamepad;
    private final IMU imu;
    private final Double[] speeds = {0.0, 0.0, 0.0};

    /**
     * Command to set the drive motors in Mecanum drive
     *
     * @param basicMecanumDrive The subsystem to pull from
     * @param gamepad The gamepad to complete the command
     */
    public MecanumDriveCommand(BasicMecanumDrive basicMecanumDrive, IMU imu, GamepadEx gamepad){
        this.m_basicMecanumDrive = basicMecanumDrive;
        this.imu = imu;
        this.gamepad = gamepad;

        addRequirements(basicMecanumDrive);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        speeds[0] = gamepad.getLeftX();
        speeds[1] = gamepad.getLeftY();
        speeds[2] = gamepad.getRightX();
        m_basicMecanumDrive.mecanumDrive(speeds, imu, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

}
