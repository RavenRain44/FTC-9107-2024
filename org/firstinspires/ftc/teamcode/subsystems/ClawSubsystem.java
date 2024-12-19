package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    // Creating the servo objects
    private final Servo wristServo;
    private final Servo clawServo;

    // Arm Units
    public double targetClawPosition = 0;

    // Dashboard
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    /**
     * [BETA]
     * <p>
     * Claw Subsystem to control the wrist and claw.
     * </p>
     * @param wristServo Servo to control the angle of the wrist
     * @param clawServo  Servo to control the claw
     */
    public ClawSubsystem(
            Servo wristServo,
            Servo clawServo) {
        this.wristServo = wristServo;
        this.clawServo = clawServo;
    }



    /**
     * Sets the claw servo to the open position set in <code>Constants.java</code>.
     *
     * @see org.firstinspires.ftc.teamcode.Constants#OPEN_POSITION
     * @see Servo#setPosition(double)
     */
    public void openClaw() {
        this.clawServo.setPosition(OPEN_POSITION);
    }


    /**
     * Sets the claw servo to the closed position as set in <code>Constants.java</code>.
     *
     * @see org.firstinspires.ftc.teamcode.Constants#CLOSE_POSITION
     * @see Servo#setPosition(double)
     */
    public void closeClaw() {
        this.clawServo.setPosition(CLOSE_POSITION);
    }

    public boolean clawIsClosed() {
        return this.clawServo.getPosition() <= CLOSE_POSITION + 0.01 && this.clawServo.getPosition() >= CLOSE_POSITION - 0.01;
    }

    public boolean clawIsOpen() {
        return this.clawServo.getPosition() <= OPEN_POSITION + 0.01 && this.clawServo.getPosition() >= OPEN_POSITION - 0.01;
    }


    /**
     * Sets the claw servo to the specified degree.
     *
     * @param power A value between 0-1
     * @see Servo#setPosition(double)
     */
    public void setClawToPosition(double power) {
        this.clawServo.setPosition(power);
        this.targetClawPosition = power;
    }

    public void clawBack() {
        this.wristServo.setPosition(OUTTAKE_WRIST_ANGLE);
    }

    public void clawIntake() {
        this.wristServo.setPosition(INTAKE_WRIST_ANGLE);
    }

    public void setWristAngle(double a) {
        this.wristServo.setPosition(a * ((WRIST_NINETY - WRIST_ZERO) / 90) + WRIST_ZERO);
    }

    public void setApparentWristAngle(double a, ArmSubsystem armSubsystem) {
        setWristAngle(a - armSubsystem.getArmAngle());
    }

    public void setAppWristAngleZero(ArmSubsystem armSubsystem) {
        setWristAngle(-armSubsystem.getArmAngle());
    }

    public double getWristAngle() {
        return (this.wristServo.getPosition() - WRIST_ZERO) / (((WRIST_NINETY - WRIST_ZERO) / 90));
    }

    /*
     * ACTIONS
     */

    public class ClawOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            openClaw();
            return false;
        }
    }

    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            closeClaw();
            return false;
        }
    }

    public class WristIntake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawIntake();
            return false;
        }
    }

    public class WristOuttake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawBack();
            return false;
        }
    }

    public Action clawOpen () {
        return new ClawOpen();
    }

    public Action clawClose() {
        return new ClawClose();
    }

    public Action wristIntake() {
        return new WristIntake();
    }

    public Action wristOuttake() {
        return new WristOuttake();
    }

    @Override
    public void periodic() {
        // Logging
        TelemetryPacket clawData = new TelemetryPacket();
        clawData.put("Claw angle", clawServo.getPosition());
        clawData.put("Claw Open?", (clawServo.getPosition() == OPEN_POSITION));
        clawData.put("Wrist angle", getWristAngle());
        dashboard.sendTelemetryPacket(clawData);
    }
}