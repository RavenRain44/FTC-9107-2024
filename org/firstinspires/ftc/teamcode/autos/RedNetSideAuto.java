package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous (name = "BasicAuto", group = "Autonomous")
public class RedNetSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-49.25, -64, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Pose2d block1pos = new Pose2d(-48.5, -33.35, Math.toRadians(90)); // OO0
        Pose2d block2pos = new Pose2d(-58.5, -33.35, Math.toRadians(90)); // O0O
        Pose2d block3pos = new Pose2d(-69, -33.35, Math.toRadians(180));  // 0OO
        Pose2d netZonePos = new Pose2d(-55.5, -54, Math.toRadians(45)); // Angle of station for end heading of robot
        Pose2d observationZonePos = new Pose2d(49.675, -64, Math.toRadians(0));

        int visionOutputPosition = 1;

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

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryAction = tab1.build();

        Actions.runBlocking(new SequentialAction(
                trajectoryAction
        ));
    }
}
