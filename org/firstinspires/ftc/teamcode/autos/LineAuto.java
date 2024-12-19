package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous (name = "LineAuto", group = "Autonomous")
public class LineAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(42.125, 64, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Vector2d block1pos = new Vector2d(54, 35);
        Pose2d block2pos = new Pose2d(0, 64, Math.toRadians(180));
        Pose2d block3pos = new Pose2d(65, 25, Math.toRadians(0));
        Pose2d netZonePos = new Pose2d(56.75, 53, Math.toRadians(225)); // Angle of station for end heading of robot
        Pose2d observationZonePos = new Pose2d(-40, 65, Math.toRadians(180));

        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(180) // Angle of Original Heading for the end of spline
                .splineToConstantHeading(block2pos.component1(), Math.toRadians(180)); // Angle Relative to the field for the angle of the robot

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryAction = tab1.build();

        Actions.runBlocking(new SequentialAction(
                trajectoryAction
        ));
    }
}
