package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="circleAuto", group = "Autonomous")
public class CircleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // starting position
        Pose2d initialPose = new Pose2d(0, -65.6, Math.toRadians(90));

        // drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder trajectory = drive.actionBuilder(initialPose)
                .lineToY(-50)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(50, 0), Math.toRadians(90))
                .splineTo(new Vector2d(0, 50), Math.toRadians(180))
                .splineTo(new Vector2d(-50, 0), Math.toRadians(270))
                .splineTo(new Vector2d(0, -50), Math.toRadians(0))
                .waitSeconds(1)
                .turn(Math.toRadians(90))
                .lineToY(-65.6);

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryAction = trajectory.build();

        // run auto
        Actions.runBlocking(new SequentialAction(
                trajectoryAction
        ));
    }
}
