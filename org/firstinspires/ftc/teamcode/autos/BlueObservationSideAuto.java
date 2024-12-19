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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

@Config
@Autonomous (name = "BlueObservationSideAuto", group = "Autonomous")
public class BlueObservationSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-12.5, 61, Math.toRadians(90));
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

        Vector2d block1pos = new Vector2d(-54, 35);
        Pose2d block2pos = new Pose2d(-65, 35, Math.toRadians(270));
        Pose2d block3pos = new Pose2d(-65, 25, Math.toRadians(0));
        Pose2d observationZonePos = new Pose2d(-40, 65, Math.toRadians(90));
        Pose2d rungpos = new Pose2d(-12.5, 37.5, Math.toRadians(90));
        Pose2d backUpPos = new Pose2d(-12.5, 40, Math.toRadians(90));

        int visionOutputPosition = 1;

        TrajectoryActionBuilder firstRung = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(rungpos.component1(), Math.toRadians(90));

        TrajectoryActionBuilder backUp = drive.actionBuilder(rungpos)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(backUpPos.component1(), Math.toRadians(270));

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(0)
                .splineToConstantHeading(block1pos, Math.toRadians(300))
                .waitSeconds(1)
                .setTangent(90)
                .splineToLinearHeading(observationZonePos, Math.toRadians(270))
                .waitSeconds(1)
                .setTangent(270)
                .splineToLinearHeading(block2pos, Math.toRadians(270))
                .waitSeconds(1)
                .setTangent(180)
                .splineToLinearHeading(observationZonePos, Math.toRadians(270))
                .waitSeconds(1)
                .setTangent(270)
                .splineToLinearHeading(block3pos, Math.toRadians(270))
                .waitSeconds(1)
                .setTangent(180)
                .splineToLinearHeading(observationZonePos, Math.toRadians(270))
                .waitSeconds(1)
                .setTangent(270)
                .splineToLinearHeading(rungpos, Math.toRadians(90))
                .waitSeconds(3)
                .setTangent(90)
                .splineToLinearHeading(observationZonePos, Math.toRadians(270))
                .waitSeconds(1)
                .setTangent(270)
                .splineToLinearHeading(rungpos, Math.toRadians(90))
                .waitSeconds(3)
                .setTangent(90)
                .splineToLinearHeading(observationZonePos, Math.toRadians(270))
                .waitSeconds(1)
                .setTangent(270)
                .splineToLinearHeading(rungpos, Math.toRadians(90))
                .waitSeconds(3)
                .setTangent(270)
                .splineToLinearHeading(observationZonePos, Math.toRadians(180));

        clawSubsystem.closeClaw();
        clawSubsystem.clawBack();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryAction = tab1.build();
        Action FirstRung = firstRung.build();
        Action BackUp = backUp.build();

        Actions.runBlocking(new SequentialAction(
                FirstRung,
                armSubsystem.raiseArmToMax(),
                new ParallelAction(
                        clawSubsystem.clawOpen(),
                        BackUp,
                        new SleepAction(0.2)
                ),
                armSubsystem.lowerArmToMin()
        ));
    }
}