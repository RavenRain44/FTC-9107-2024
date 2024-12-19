package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Config
@Autonomous (name = "BlueNetSideAuto", group = "Autonomous")
public class BlueNetSideAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(42.125, 64, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmSubsystem armSubsystem = new ArmSubsystem(
                new Motor(hardwareMap, "leftArm", Motor.GoBILDA.RPM_30),
                new Motor(hardwareMap, "rightArm", Motor.GoBILDA.RPM_30),
                new Motor(hardwareMap, "viperSlide", Motor.GoBILDA.RPM_117)
        );

        ClawSubsystem clawSubsystem = new ClawSubsystem(
                hardwareMap.get(Servo.class, "wrist"),
                hardwareMap.get(Servo.class, "claw")
        );

        Pose2d block1pos = new Pose2d(52, 37.5, Math.toRadians(270));
        Pose2d block2pos = new Pose2d(62, 34.5, Math.toRadians(270));
        Pose2d block3pos = new Pose2d(63, 24, Math.toRadians(0));
        Pose2d netZonePos = new Pose2d(56.75, 53, Math.toRadians(225)); // Angle of station for end heading of robot
        Pose2d intoNetZonePos1 = new Pose2d(60, 57.25, Math.toRadians(225)); // Angle of station for end heading of robot
        Pose2d intoNetZonePos2 = new Pose2d(60.5, 57.75, Math.toRadians(225)); // Angle of station for end heading of robot
        Pose2d intoNetZone = new Pose2d(62.5, 58.75, Math.toRadians(225));
        Pose2d observationZonePos = new Pose2d(-40, 65, Math.toRadians(180));

        int visionOutputPosition = 1;

        TrajectoryActionBuilder toBlock1 = drive.actionBuilder(netZonePos)
                .setTangent(270) // Angle of Original Heading for the end of spline
                .splineToLinearHeading(block1pos, Math.toRadians(270)); // Angle Relative to the field for the angle of the robot

        TrajectoryActionBuilder toBlock2 = drive.actionBuilder(netZonePos)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(block2pos, Math.toRadians(270));

        TrajectoryActionBuilder toBlock3 = drive.actionBuilder(netZonePos)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(block3pos, Math.toRadians(0)); // Angle Relative to the field for the end of the spline

        TrajectoryActionBuilder toNetZone1 = drive.actionBuilder(block1pos)
                .setTangent(Math.toRadians(90)) // Angle relative to field for start of spline
                .splineToLinearHeading(netZonePos, Math.toRadians(45)); // Angle Relative to the field for the end of the spline

        TrajectoryActionBuilder toNetZone2 = drive.actionBuilder(block1pos)
                .setTangent(Math.toRadians(90)) // Angle relative to field for start of spline
                .splineToLinearHeading(netZonePos, Math.toRadians(45)); // Angle Relative to the field for the end of the spline

        TrajectoryActionBuilder toNetZone3 = drive.actionBuilder(block1pos)
                .setTangent(Math.toRadians(90)) // Angle relative to field for start of spline
                .splineToLinearHeading(netZonePos, Math.toRadians(45)); // Angle Relative to the field for the end of the spline

        TrajectoryActionBuilder toNetZone = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(315)) // Angle relative to field for start of spline
                .splineToLinearHeading(netZonePos, Math.toRadians(180)); // Angle Relative to the field for the end of the spline

        TrajectoryActionBuilder toObservationZone = drive.actionBuilder(netZonePos)
                .setTangent(Math.toRadians(180)) // Angle relative to field for start of spline
                .splineToLinearHeading(observationZonePos, Math.toRadians(180)); // Angle Relative to the field for the end of the spline

        TrajectoryActionBuilder inchToNet = drive.actionBuilder(netZonePos)
                .setTangent(Math.toRadians(225)) // Angle relative to field for start of spline
                .splineToLinearHeading(intoNetZone, Math.toRadians(45)); // Angle Relative to the field for the end of the spline
        TrajectoryActionBuilder inchToNet1 = drive.actionBuilder(netZonePos)
                .setTangent(Math.toRadians(225)) // Angle relative to field for start of spline
                .splineToLinearHeading(intoNetZonePos1, Math.toRadians(45)); // Angle Relative to the field for the end of the spline
        TrajectoryActionBuilder inchToNet2 = drive.actionBuilder(netZonePos)
                .setTangent(Math.toRadians(225)) // Angle relative to field for start of spline
                .splineToLinearHeading(intoNetZonePos2, Math.toRadians(45)); // Angle Relative to the field for the end of the spline

        TrajectoryActionBuilder inchFromNet = drive.actionBuilder(intoNetZone)
                .setTangent(Math.toRadians(45)) // Angle relative to field for start of spline
                .splineToLinearHeading(netZonePos, Math.toRadians(225)); // Angle Relative to the field for the end of the spline
        TrajectoryActionBuilder inchFromNet1 = drive.actionBuilder(intoNetZonePos1)
                .setTangent(Math.toRadians(45)) // Angle relative to field for start of spline
                .splineToLinearHeading(netZonePos, Math.toRadians(225)); // Angle Relative to the field for the end of the spline
        TrajectoryActionBuilder inchFromNet2 = drive.actionBuilder(intoNetZonePos2)
                .setTangent(Math.toRadians(45)) // Angle relative to field for start of spline
                .splineToLinearHeading(netZonePos, Math.toRadians(225)); // Angle Relative to the field for the end of the spline

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(0) // Angle of Original Heading for the end of spline
                .splineToConstantHeading(block1pos.component1(), Math.toRadians(300)) // Angle Relative to the field for the angle of the robot
                .waitSeconds(1)
                .setTangent(Math.toRadians(90)) // Angle relative to field for start of spline
                .splineToLinearHeading(netZonePos, Math.toRadians(45)) // Angle Relative to the field for the end of the spline
                .waitSeconds(1)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(block2pos, Math.toRadians(270))
                .waitSeconds(1)
                .setTangent(Math.toRadians(90)) // Angle relative to field for start of spline
                .splineToLinearHeading(netZonePos, Math.toRadians(45)) // Angle Relative to the field for the end of the spline
                .waitSeconds(1)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(block3pos, Math.toRadians(0)) // Angle Relative to the field for the end of the spline
                .waitSeconds(1)
                .setTangent(Math.toRadians(90)) // Angle relative to field for start of spline
                .splineToLinearHeading(netZonePos, Math.toRadians(45)) // Angle Relative to the field for the end of the spline
                .waitSeconds(1)
                .setTangent(Math.toRadians(180)) // Angle relative to field for start of spline
                .splineToLinearHeading(observationZonePos, Math.toRadians(180)); // Angle Relative to the field for the end of the spline

        clawSubsystem.closeClaw();
        clawSubsystem.clawBack();

        waitForStart();

        Actions.runBlocking(clawSubsystem.clawClose());

        if (isStopRequested()) return;

        Action trajectoryAction = tab1.build();
        Action ToBlock1 = toBlock1.build();
        Action ToBlock2 = toBlock2.build();
        Action ToBlock3 = toBlock3.build();
        Action ToNetZone1 = toNetZone1.build();
        Action ToNetZone2 = toNetZone2.build();
        Action ToNetZone3 = toNetZone3.build();
        Action ToNetZone = toNetZone.build();
        Action ToObservationZone = toObservationZone.build();
        Action InchToNet = inchToNet.build();
        Action InchToNet1 = inchToNet1.build();
        Action InchToNet2 = inchToNet2.build();
        Action InchFromNet = inchFromNet.build();
        Action InchFromNet1 = inchFromNet1.build();
        Action InchFromNet2 = inchFromNet2.build();

        Actions.runBlocking(new SequentialAction(
                ToNetZone,
                new ParallelAction(
                        armSubsystem.raiseArmToMax(),
                        clawSubsystem.wristIntake(),
                        new SleepAction(0.2),
                        armSubsystem.extendSlideToMax()
                ),
                InchToNet,
                clawSubsystem.wristOuttake(),
                new SleepAction(0.2),
                clawSubsystem.clawOpen(),
                new SleepAction(0.2),
                clawSubsystem.clawClose(),
                new SleepAction(0.1),
                InchFromNet,
                armSubsystem.retractSlideToMin(),
                armSubsystem.lowerArmToMin(),
                ToBlock1,
                new ParallelAction(
                        clawSubsystem.clawOpen(),
                        clawSubsystem.wristIntake(),
                        new SleepAction(0.3)
                ),
                clawSubsystem.clawClose(),
                new SleepAction(0.1),

                ToNetZone1,
                new ParallelAction(
                        armSubsystem.raiseArmToMax(),
                        armSubsystem.extendSlideToMax()
                ),
                InchToNet1,
                clawSubsystem.wristOuttake(),
                new SleepAction(0.2),
                clawSubsystem.clawOpen(),
                new SleepAction(0.2),
                clawSubsystem.clawClose(),
                new SleepAction(0.1),
                InchFromNet1,
                armSubsystem.retractSlideToMin(),
                armSubsystem.lowerArmToMin(),
                ToBlock2,
                new ParallelAction(
                        clawSubsystem.clawOpen(),
                        clawSubsystem.wristIntake(),
                        new SleepAction(0.3)
                ),
                clawSubsystem.clawClose(),
                new SleepAction(0.2),

                ToNetZone2,
                new ParallelAction(
                        armSubsystem.raiseArmToMax(),
                        armSubsystem.extendSlideToMax()
                ),
                InchToNet2,
                clawSubsystem.wristOuttake(),
                new SleepAction(0.2),
                clawSubsystem.clawOpen(),
                new SleepAction(0.2),
                clawSubsystem.clawClose(),
                new SleepAction(0.1),
                InchFromNet2,
                armSubsystem.retractSlideToMin(),
                armSubsystem.lowerArmToMin(),
//                ToBlock3,
//                clawSubsystem.clawOpen(),
//                new SleepAction(0.5),
//                clawSubsystem.wristIntake(),
//                new SleepAction(0.5),
//                clawSubsystem.clawClose(),
//                new SleepAction(0.5),
//                clawSubsystem.wristOuttake(),
//                new SleepAction(0.5),
//                ToNetZone3,
//                armSubsystem.raiseArmToMax(), // if these don't work try the sleep action
//                armSubsystem.extendSlideToMax(),
//                InchToNet,
//                clawSubsystem.clawOpen(),
//                new SleepAction(0.5),
//                clawSubsystem.clawClose(),
//                new SleepAction(0.5),
//                InchFromNet,
//                armSubsystem.retractSlideToMin(),
//                armSubsystem.lowerArmToMin(),
                ToObservationZone
        ));
    }
}
