package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BasicMecanumDrive extends SubsystemBase {

    private MecanumDrive mecanumDrive;
    private Telemetry telemetry;


    /**
     * Example of a basic mecanum drive system.
     *
     * @param frontLeft  Front Left Motor
     * @param frontRight Front Right Motor
     * @param backLeft   Back Left Motor
     * @param backRight  Back Right Motor
     */
    public BasicMecanumDrive(
            Motor frontLeft, Motor frontRight,
            Motor backLeft, Motor backRight,
            Telemetry telemetry) {

        mecanumDrive = new MecanumDrive(false, frontLeft, frontRight, backLeft, backRight);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontLeft.setInverted(true);
        backLeft.setInverted(true);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {

    }

    /**
     * This method drives the robot either RobotCentric or FieldCentric
     *
     * @param inputs Order: Horizontal speed, Lateral speed, Turn speed
     * @param imu The IMU of the robot
     * @param fieldCentric Boolean where the value true makes the movement robot centric
     */
    public void mecanumDrive(Double[] inputs, IMU imu, boolean fieldCentric) {

        if (fieldCentric) {
            mecanumDrive.driveFieldCentric(
                    inputs[0],
                    inputs[1],
                    inputs[2],
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
                    true);
        } else {
            mecanumDrive.driveRobotCentric(
                    inputs[0],
                    inputs[1],
                    inputs[2],
                    true);
        }
    }

    public void zeroHeading(IMU imu) {
        imu.resetYaw();
    }
}