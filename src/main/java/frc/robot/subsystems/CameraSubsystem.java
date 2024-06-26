package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.HttpCamera;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class CameraSubsystem extends SubsystemBase {
  /** Creates a new CameraSubsystem. */
  //private UsbCamera usbCamera;
  private HttpCamera romiCamera;
  private CvSink cvSink;
  private CvSource outputStream;
  
  private BooleanLogEntry cameraInitializedLog;
  private StringLogEntry logEntry;
  private boolean cameraInitialized = false;

   // New member variables for AprilTag
   private AprilTagDetector detector;
   private AprilTagPoseEstimator estimator;
   private NetworkTable tagsTable;
   private IntegerArrayPublisher pubTags;
   private Scalar outlineColor = new Scalar(0, 255, 0);
   private Scalar crossColor = new Scalar(0, 0, 255);

   private Drivetrain drivetrain;  // Reference to the drivetrain subsystem
  
  public CameraSubsystem(Drivetrain drivetrain) {
    this.drivetrain = drivetrain; // Initialize the drivetrain
    // Start the DataLogManager
    DataLogManager.start();
        
    // Get the log instance
    var log = DataLogManager.getLog();
    
    // Create log entries
    cameraInitializedLog = new BooleanLogEntry(log, "/camera/initialized");
    logEntry = new StringLogEntry(log, "/camera/log");
    
    initializeCamera();

    // New AprilTag initialization
    initializeAprilTag();
}

  private void initializeCamera() {
    try {
        // Initialize the USB camera
        //usbCamera = CameraServer.startAutomaticCapture();
        //usbCamera.setResolution(640, 480);
        romiCamera = new HttpCamera("RomiCamera", "http://10.0.0.2:1181/stream.mjpg");
        cvSink = CameraServer.getVideo(romiCamera);
        outputStream = CameraServer.putVideo("RomiProcessed", 640, 480);

        logEntry.append("USB Camera initialized");

        // Stream the camera to the Shuffleboard
        Shuffleboard.getTab("Camera")
                    //.add("USB Camera", usbCamera)
                    .add("Romi Camera", outputStream)
                    .withWidget(BuiltInWidgets.kCameraStream);
        logEntry.append("Romi Camera Initialized");
        cameraInitialized = true;
        cameraInitializedLog.append(true);
    } catch (Exception e) {
        logEntry.append("Failed to initialize Romi Camera: " + e.getMessage());
        cameraInitialized = false;
        cameraInitializedLog.append(false);
    }
}
private void initializeAprilTag() {
  detector = new AprilTagDetector();
  detector.addFamily("tag36h11", 1);
  var poseEstConfig = new AprilTagPoseEstimator.Config(
      0.1651, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
  estimator = new AprilTagPoseEstimator(poseEstConfig);
  tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
  pubTags = tagsTable.getIntegerArrayTopic("tags").publish();
}

  @Override
  public void periodic() {
    // Reinitialize the camera if it wasn't successfully initialized
    if (!cameraInitialized) {
      logEntry.append("Attempting to reinitialize Romi Camera");
      initializeCamera();
      return;
    }
  Mat source = new Mat();
  if (cvSink.grabFrame(source) == 0) {
      logEntry.append("Failed to grab frame from Romi Camera:" + cvSink.getError());
      return;
   }
   // Do any processing with the source frame if needed

    Mat grayMat = new Mat();
    Imgproc.cvtColor(source, grayMat, Imgproc.COLOR_RGB2GRAY);
    AprilTagDetection[] detections = detector.detect(grayMat);
    ArrayList<Long> tags = new ArrayList<>();

    for (AprilTagDetection detection : detections) {
        tags.add((long) detection.getId());
        for (int i = 0; i <= 3; i++) {
            int j = (i + 1) % 4;
            Point pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
            Point pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
            Imgproc.line(source, pt1, pt2, outlineColor, 2);
        }
        Point center = new Point(detection.getCenterX(), detection.getCenterY());
        int ll = 10;
        Imgproc.line(source, new Point(center.x - ll, center.y), new Point(center.x + ll, center.y), crossColor, 2);
        Imgproc.line(source, new Point(center.x, center.y - ll), new Point(center.x, center.y + ll), crossColor, 2);
        Imgproc.putText(source, Integer.toString(detection.getId()), new Point(center.x + ll, center.y), Imgproc.FONT_HERSHEY_SIMPLEX, 1, crossColor, 3);

        Transform3d pose = estimator.estimate(detection);
        Rotation3d rot = pose.getRotation();
        tagsTable.getEntry("pose_" + detection.getId()).setDoubleArray(new double[]{pose.getX(), pose.getY(), pose.getZ(), rot.getX(), rot.getY(), rot.getZ()});

        // Generate navigation commands based on the tag pose
        navigateToTag(pose);
    }

    pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());
    outputStream.putFrame(source);  // Ensure the frame is put to the output stream
  }
  // Add the navigation methods after the periodic method
  private void navigateToTag(Transform3d tagPose) {
    double targetX = tagPose.getX();
    double targetY = tagPose.getY();
    double targetZ = tagPose.getZ();

    // Calculate desired movement based on tag position
    double forwardSpeed = calculateForwardSpeed(targetZ);
    double turnSpeed = calculateTurnSpeed(targetX);

    // Command the drivetrain to move towards the tag
    drivetrain.arcadeDrive(forwardSpeed, turnSpeed);
}

private double calculateForwardSpeed(double distance) {
    // Implement a PID controller or a simple proportional control for forward speed
    double kP = 0.1;  // Proportional gain, tune as needed
    return kP * distance;
}

private double calculateTurnSpeed(double offsetX) {
    // Implement a PID controller or a simple proportional control for turn speed
    double kP = 0.1;  // Proportional gain, tune as needed
    return kP * offsetX;
}

}
