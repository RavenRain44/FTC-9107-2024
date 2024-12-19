package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.opengl.models.Geometry;
import org.firstinspires.ftc.teamcode.commands.DefaultState;
import org.firstinspires.ftc.teamcode.commands.IntakeState;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeState;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BasicMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@TeleOp(name= "DeepOpMode", group="Test")
public class DeepOpMode extends CommandOpMode {
    private BasicMecanumDrive m_basicMecanumDrive;
    //private VisionSubsystem m_visionSubsystem;
    private ArmSubsystem m_armSubsystem;
    private ClawSubsystem m_clawSubsystem;

    @Override
    public void initialize() {
        /*
         * INITIALIZE GAMEPADS
         */
        final GamepadEx driver = new GamepadEx(gamepad1);
        final GamepadEx mech = new GamepadEx(gamepad2);


        /*
         * INITIALIZE IMU AND VISION
         */
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        //final CameraStreamProcessor processor = new CameraStreamProcessor();


        /*
         * INITIALIZE BUTTONS
         */
        // Driver Buttons
        Button resetHeading = new GamepadButton(driver, GamepadKeys.Button.START);
        Button intakeState = new GamepadButton(driver, GamepadKeys.Button.A);
        Button outtakeState = new GamepadButton(driver, GamepadKeys.Button.Y);

        // Mech Buttons
        Button liftArm = new GamepadButton(mech, GamepadKeys.Button.Y);
        Button lowerArm = new GamepadButton(mech, GamepadKeys.Button.A);
        Button extendSlide = new GamepadButton(mech, GamepadKeys.Button.DPAD_UP);
        Button retractSlide = new GamepadButton(mech, GamepadKeys.Button.DPAD_DOWN);
        Button resetEncoders = new GamepadButton(mech, GamepadKeys.Button.X);



        /*
         * SUBSYSTEMS
         */
        m_basicMecanumDrive = new BasicMecanumDrive(
                new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312),
                telemetry
        );

//        m_visionSubsystem = new VisionSubsystem(
//                hardwareMap.get(WebcamName.class, "webcam"),
//                processor
//        );

        m_armSubsystem = new ArmSubsystem(
                new Motor(hardwareMap, "leftArm", Motor.GoBILDA.RPM_30),
                new Motor(hardwareMap, "rightArm", Motor.GoBILDA.RPM_30),
                new Motor(hardwareMap, "viperSlide", Motor.GoBILDA.RPM_117)
        );
        m_armSubsystem.resetEncoders();

        m_clawSubsystem = new ClawSubsystem(
                hardwareMap.get(Servo.class, "wrist"),
                hardwareMap.get(Servo.class, "claw")
        );


        // Default Commands
        m_basicMecanumDrive.setDefaultCommand(new MecanumDriveCommand(m_basicMecanumDrive, imu, driver));
        m_armSubsystem.setDefaultCommand(new DefaultState(m_armSubsystem, m_clawSubsystem));

        // Vision
//        FtcDashboard.getInstance().startCameraStream(processor, 30);


        /*
         * CONTROLS
         */
        // Driver Controls
        resetHeading.whenPressed(new InstantCommand(() -> m_basicMecanumDrive.zeroHeading(imu)));
        intakeState.whenPressed(new IntakeState(m_armSubsystem, m_clawSubsystem, driver));
        outtakeState.whenPressed(new OuttakeState(m_armSubsystem, m_clawSubsystem, driver, mech));

        // Mech Controls
//        extendSlide.whenHeld(new InstantCommand(() -> m_armSubsystem.setSlidePower(0.8)))
//                .whenReleased(new InstantCommand(() -> m_armSubsystem.setSlidePower(0)));
        retractSlide.whenHeld(new InstantCommand(() -> m_armSubsystem.setSlidePower(-0.8)))
                .whenReleased(new InstantCommand(() -> m_armSubsystem.setSlidePower(0)));
        liftArm.whenHeld(new InstantCommand(() -> m_armSubsystem.armRawPower(0.5)))
                .whenReleased(new InstantCommand(() -> m_armSubsystem.armRawPower(0)));
        lowerArm.whenHeld(new InstantCommand(() -> m_armSubsystem.armRawPower(-0.5)))
                .whenReleased(new InstantCommand(() -> m_armSubsystem.armRawPower(0)));
        resetEncoders.whenPressed(new InstantCommand(() -> m_armSubsystem.resetEncoders()));

    }


}
