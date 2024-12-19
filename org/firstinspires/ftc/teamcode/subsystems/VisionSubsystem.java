package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.CameraStreamProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionSubsystem extends SubsystemBase {
    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public Double[] closePose = {0.0, 0.0, 0.0, 0.0};

    public VisionSubsystem(WebcamName webcam, CameraStreamProcessor cameraStreamProcessor) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .addProcessor(cameraStreamProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public boolean isAprilTagPresent() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Tags Detected", tagProcessor.getDetections().size());
        dashboard.sendTelemetryPacket(packet);
        return !tagProcessor.getDetections().isEmpty();
    }

    public Double[] process() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Tags Detected", tagProcessor.getDetections().size());
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            packet.put("ID", tag.id);
            packet.put("X", tag.ftcPose.x + Constants.HORIZONTAL_CAMERA_OFFSET);
            packet.put("Y", tag.ftcPose.y + Constants.LATERAL_CAMERA_OFFSET);
            packet.put("Z", tag.ftcPose.z);
            packet.put("Yaw", tag.ftcPose.yaw);
            packet.put("Bearing", tag.ftcPose.bearing);
            packet.put("Range", tag.ftcPose.range);

            closePose[0] = tag.ftcPose.x;
            closePose[1] = tag.ftcPose.y;
            closePose[2] = tag.ftcPose.yaw;
            closePose[3] = tag.ftcPose.range;

            closePose[0] = closePose[0] + Constants.HORIZONTAL_CAMERA_OFFSET;
            closePose[1] = closePose[1] + Constants.LATERAL_CAMERA_OFFSET;

            dashboard.sendTelemetryPacket(packet);
        }
        return this.closePose;
    }

    public int getID() {
        int id = 0;
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Tags Detected", tagProcessor.getDetections().size());
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            packet.put("ID", tag.id);
            packet.put("X", tag.ftcPose.x);
            packet.put("Y", tag.ftcPose.y);
            packet.put("Z", tag.ftcPose.z);
            packet.put("Yaw", tag.ftcPose.yaw);
            packet.put("Bearing", tag.ftcPose.bearing);
            packet.put("Range", tag.ftcPose.range);

            id = tag.id;

            dashboard.sendTelemetryPacket(packet);
        }
        return id;
    }
}
