package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class ArmSubsystem extends SubsystemBase {

    // Creating the motor objects
    private final Motor leftMotor;
    private final Motor rightMotor;

    private final Motor viperMotor;

    // Dashboard
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private double expectedArmAngle = 0;



    /**
     * [BETA]
     * <p>
     * Dual-Motor arm with Viper Slide extension.
     * As of version Alpha, everything is done on paper but yet to be tested.
     * Tweak the arm values as needed.
     * </p>
     *
     * @param leftMotor  Left motor on the dual-motor arm
     * @param rightMotor Right motor on the dual-motor arm
     * @param viperMotor Motor to control the extension of the Viper Slide
     */
    public ArmSubsystem(Motor leftMotor,
                        Motor rightMotor,
                        Motor viperMotor) {
        // Initializing objects
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.viperMotor = viperMotor;

        // Setting motor direction
        this.leftMotor.setInverted(true);
        this.viperMotor.setInverted(true);

        // Setting Brake Mode
        this.viperMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Setting the units per tick
        this.leftMotor.setDistancePerPulse(1.0 / 2557.2);
        this.rightMotor.setDistancePerPulse(1.0 /2557.2);
        this.viperMotor.setDistancePerPulse(1.0 /5600.0);

        // Setting the Position Tolerance *DOESN'T WORK*
        this.leftMotor.setPositionTolerance(0.02);
        this.rightMotor.setPositionTolerance(0.02);
        this.viperMotor.setPositionTolerance(0.02);

        // NOTE All RunModes are set in methods
    }

    /**
     * Lifts up the dual-motor arm to the max position specified in <code>Constants.java</code>.
     * As of initial implementation, the code may not work as intended.
     *
     * @see org.firstinspires.ftc.teamcode.Constants#MAX_ARM_POSITION
     * @see Motor#setTargetDistance(double)
     */
    public void moveArmToMax() {
        this.leftMotor.setTargetDistance(MAX_ARM_POSITION);
        this.rightMotor.setTargetDistance(MAX_ARM_POSITION);
        this.leftMotor.setRunMode(Motor.RunMode.PositionControl);
        this.rightMotor.setRunMode(Motor.RunMode.PositionControl);
        this.leftMotor.set(MAX_ARM_POWER);
        this.rightMotor.set(MAX_ARM_POWER);
    }


    /**
     * Sets the dual-motor arm to the specified position
     *
     * @param position  The position parameter in <code>setTargetPosition()</code>
     *
     * @see Motor#setTargetDistance(double)
     */
    public void moveArmToPosition(double position) {
        this.leftMotor.setTargetDistance(position);
        this.rightMotor.setTargetDistance(position);
        this.leftMotor.setRunMode(Motor.RunMode.PositionControl);
        this.rightMotor.setRunMode(Motor.RunMode.PositionControl);
        this.leftMotor.set(MAX_ARM_POWER);
        this.rightMotor.set(MAX_ARM_POWER);
    }


    /**
     * Sets the dual-motor arm to the original position specified in <code>Constants.java</code>.
     * As of initial implementation, the code may not work as intended.
     *
     * @see org.firstinspires.ftc.teamcode.Constants#MIN_ARM_POSITION
     * @see Motor#setTargetDistance(double)
     */
    public void moveArmToOriginalPosition() {
        this.leftMotor.setTargetDistance(MIN_ARM_POSITION);
        this.rightMotor.setTargetDistance(MIN_ARM_POSITION);
        this.leftMotor.setRunMode(Motor.RunMode.PositionControl);
        this.rightMotor.setRunMode(Motor.RunMode.PositionControl);
        this.leftMotor.set(MAX_ARM_POWER);
        this.rightMotor.set(MAX_ARM_POWER);
    }

    public void setArmToDistance(double d) {
        this.leftMotor.setTargetDistance(d);
        this.rightMotor.setTargetDistance(d);
        this.leftMotor.setRunMode(Motor.RunMode.PositionControl);
        this.rightMotor.setRunMode(Motor.RunMode.PositionControl);
        this.leftMotor.set(MAX_ARM_POWER);
        this.rightMotor.set(MAX_ARM_POWER);
    }

    /**
     * Returns a boolean that checks that both arm motors are at the position.
     *
     * @return Boolean <code>leftMotor.atTargetPosition() AND rightMotor.atTargetPosition()</code>
     */
    public boolean armAtPosition() {
        return (this.leftMotor.atTargetPosition());
    }

    public double getArmAngle() {
        return (leftMotor.getDistance() - ARM_ZERO) * (90 / (ARM_NINETY - ARM_ZERO));
    }

    public double angleToArmUnits(double a) {
        return ((a * (ARM_NINETY - ARM_ZERO)) / 90) + ARM_ZERO;
    }

    public void armIntakeMode() {
        this.leftMotor.setTargetDistance(angleToArmUnits(Math.toDegrees(Math.asin(HEIGHT / slideUnitsToIn(getViperPosition())))) / 3);
        this.rightMotor.setTargetDistance(angleToArmUnits(Math.toDegrees(Math.asin(HEIGHT / slideUnitsToIn(getViperPosition())))) / 3);
        this.leftMotor.setRunMode(Motor.RunMode.PositionControl);
        this.rightMotor.setRunMode(Motor.RunMode.PositionControl);
        this.leftMotor.set(MAX_ARM_POWER); // TODO Adjust powers so that the arm is faster than the viper slide
        this.rightMotor.set(MAX_ARM_POWER);
    }

    public void armRawPower(double p) {
        this.leftMotor.setRunMode(Motor.RunMode.RawPower);
        this.rightMotor.setRunMode(Motor.RunMode.RawPower);
        this.leftMotor.set(p);
        this.rightMotor.set(p);
    }

    /**
     * Stops the motors and applies the gravity power to counteract the force of gravity
     */
    public void stopMotors() {
        this.leftMotor.setRunMode(Motor.RunMode.RawPower);
        this.rightMotor.setRunMode(Motor.RunMode.RawPower);
        this.leftMotor.set(viperMotor.getDistance() * (Math.cos(leftMotor.getDistance()* Math.PI / 2) / 8));
        this.rightMotor.set(viperMotor.getDistance() * (Math.cos(rightMotor.getDistance()* Math.PI / 2) / 8));
    }

    /**
     * Resets the motor encoders on the robot
     */
    public void resetEncoders() {
        // Motors
        this.viperMotor.resetEncoder();
        this.leftMotor.resetEncoder();
        this.rightMotor.resetEncoder();
    }

    /**
     * Extends the Viper Slide to the max position specified in <code>Constants.java</code>.
     *
     * @see org.firstinspires.ftc.teamcode.Constants#MAX_EXTEND_POS
     * @see Motor#setTargetDistance(double)
     */
    public void extendArmToMax() {
        this.viperMotor.setTargetDistance(MAX_EXTEND_POS);
        this.viperMotor.setRunMode(Motor.RunMode.PositionControl);
        this.viperMotor.set(MAX_VIPER_POWER);
    }


    /**
     * Retracts the Viper Slide to the minimum position specified in <code>Constants.java</code>.
     *
     * @see org.firstinspires.ftc.teamcode.Constants#MIN_EXTEND_POS
     * @see Motor#setTargetDistance(double)
     */
    public void retractArmToMin() {
        this.viperMotor.setTargetDistance(MIN_EXTEND_POS);
        this.viperMotor.setRunMode(Motor.RunMode.PositionControl);
        this.viperMotor.set(MAX_VIPER_POWER);
    }


    /**
     * Extends the Viper Slide to a declared position.
     *
     * @param position Position in inches relative to starting position
     * @see Motor#setTargetDistance(double)
     */
    public void extendArmToPosition(double position) {
        this.viperMotor.setTargetDistance(position);
        this.viperMotor.setRunMode(Motor.RunMode.PositionControl);
        this.viperMotor.set(MAX_VIPER_POWER);
    }

    /**
     * Sets the slide position to intake position
     *
     * @see Motor#setTargetDistance(double)
     */
    public void prepareIntake() {
        this.viperMotor.setTargetDistance(INTAKE_SLIDE_POSITION);
        this.viperMotor.setRunMode(Motor.RunMode.PositionControl);
        this.viperMotor.set(MAX_VIPER_POWER);
    }

    /**
     * Gets the current position of the viper slide in the distance units
     *
     * @see Motor#setDistancePerPulse(double)
     */
    public double getViperPosition() {
        return this.viperMotor.getDistance();
    }

    /**
     * Returns if the viper slide is at the requested position
     *
     * @return Viper slide at target position?
     */
    public boolean slideAtPosition() {
        return this.viperMotor.atTargetPosition();
    }

    public double slideUnitsToIn(double u) {
        return u * (VIPER_MAX_LENGTH - VIPER_MIN_LENGTH) + VIPER_MIN_LENGTH;
    }

    public void setSlidePower(double p) {
        this.viperMotor.setRunMode(Motor.RunMode.RawPower);
        this.viperMotor.set(p);
    }

    /**
     * Stops the viper slide motor
     */
    public void stopSlide() {
        this.viperMotor.set(0);
    }

    /**
     * Gets the current position of the arm motors in the distance units
     *
     * @see Motor#setDistancePerPulse(double)
     */
    public double getArmPosition() {
        return this.leftMotor.getDistance();
    }

    public void setArmSpeed(double speed) {
        this.leftMotor.set(speed);
        this.rightMotor.set(speed);
    }

    /*
     * ACTIONS
     * TODO Create to the high rung
     *  TODO Documentation
     */

    /**
     * Action to raise the arm to max position
     */
    public class RaiseArmToMax implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                setArmToDistance(MAX_ARM_POSITION - 0.02);
                initialized = true;
            }

            boolean atTargetPos = getArmPosition() >= MAX_ARM_POSITION - 0.02;
            packet.put("At Target Position", atTargetPos);
            if (atTargetPos) {
                leftMotor.stopMotor();
                rightMotor.stopMotor();
            }
            return !atTargetPos;
        }
    }

    public class LowerArmToMin implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                setArmToDistance(MIN_ARM_POSITION);
                initialized = true;
            }

            boolean atTargetPos = getArmPosition() <= MIN_ARM_POSITION + 0.01;
            packet.put("At Target Position", atTargetPos);
            if (atTargetPos) {
                leftMotor.stopMotor();
                rightMotor.stopMotor();
            }
            return !atTargetPos;
        }
    }

    public class ExtendSlideToMax implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                viperMotor.setTargetDistance(MAX_EXTEND_POS);
                viperMotor.setRunMode(Motor.RunMode.PositionControl);
                viperMotor.set(1);
                initialized = true;
            }
            boolean atTargetPos = getViperPosition() >= MAX_EXTEND_POS - 0.02;
            packet.put("At Target Position", atTargetPos);
            if (atTargetPos) {
                viperMotor.stopMotor();
            }
            return !atTargetPos;
        }
    }

    public class RetractSlideToMin implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                viperMotor.setTargetDistance(MIN_EXTEND_POS);
                viperMotor.setRunMode(Motor.RunMode.PositionControl);
                viperMotor.set(1);
                initialized = true;
            }

            boolean atTargetPos = getViperPosition() <= MIN_EXTEND_POS + 0.03;
            packet.put("At Target Position", atTargetPos);
            if (atTargetPos) {
                viperMotor.stopMotor();
            }
            return !atTargetPos;
        }
    }

    /**
     * Returns a new RaiseArmToMax Action for autonomous
     *
     * @return RaiseArmToMax Action
     */
    public Action raiseArmToMax() {
        return new RaiseArmToMax();
    }

    /**
     * Returns a new LowerArmToMin Action for autonomous
     *
     * @return LowerArmToMin Action
     */
    public Action lowerArmToMin() {
        return new LowerArmToMin();
    }

    /**
     * Returns a new ExtendSlideToMax Action for autonomous
     *
     * @return ExtendSlideToMax Action
     */
    public Action extendSlideToMax() {
        return new ExtendSlideToMax();
    }

    /**
     * Returns a new RetractSlideToMin Action for autonomous
     *
     * @return RetractSlideToMin Action
     */
    public Action retractSlideToMin() {
        return new RetractSlideToMin();
    }

    @Override
    public void periodic() {
        // Logging
        TelemetryPacket armData = new TelemetryPacket();
        armData.put("VS Ticks", viperMotor.getCurrentPosition());
        armData.put("VS Distance", slideUnitsToIn(viperMotor.getDistance()));
        armData.put("VS at length", viperMotor.atTargetPosition());
        armData.put("Arm Angle", getArmAngle());
        armData.put("Intake EAA", expectedArmAngle);
        armData.put("Expected Arm Angle", Math.toDegrees(Math.asin(HEIGHT / slideUnitsToIn(getViperPosition()))));
        armData.put("Expected Arm Distance", angleToArmUnits(Math.toDegrees(Math.asin(HEIGHT / slideUnitsToIn(getViperPosition())))));
        armData.put("Arm Ticks", leftMotor.getCurrentPosition());
        armData.put("Left Arm Distance", leftMotor.getDistance());
        armData.put("Right Arm Distance", rightMotor.getDistance());
        armData.put("Arm at position", leftMotor.atTargetPosition());
        dashboard.sendTelemetryPacket(armData);
    }
}