/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.CvSink;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;

import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;

import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Timer;

import frc.robot.subsystems.GripPipeline;

import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * Add your docs here.
 */
public class VisionProcess extends Thread {
  static UsbCamera camera;
  public static double targetWidth = 13.0; // physical width of target (inches)
  public static double targetHeight = 8.0; // physical height of target (inches)
  public static double distanceToFillWidth = 16.5; // measured distance where target just fills screen (width)
  public static double distanceToFillHeight = 13.5; // measured distance where target just fills screen (height)
  public static double cameraFovW = 2 * Math.atan(0.5 * targetWidth / distanceToFillWidth); // 41.8 degrees
  public static double cameraFovH = 2 * Math.atan(0.5 * targetHeight / distanceToFillHeight); // 33.0 degrees

  public static double imageWidth = 320;
  public static double imageHeight = 240;
  // multiply these factors by target screen projection (pixels) to get distance
  double distanceFactorWidth = 0.5 * targetWidth * imageWidth / Math.tan(cameraFovW / 2.0);
  double distanceFactorHeight = 0.5 * targetHeight * imageHeight / Math.tan(cameraFovH / 2.0);
  // multiply these factors by target center offset (pixels) to get horizontal and
  // vertical angle offsets
  double angleFactorWidth = Math.toDegrees(cameraFovW) / imageWidth;
  double angleFactorHeight = Math.toDegrees(cameraFovH) / imageHeight;
  // expected width/height ratio

  boolean useSerial = false;

  SerialPort ultrasonicPort;
  DigitalOutput rangingPin;

  AnalogInput rangefinder = new AnalogInput(1);
  double targetAspectRatio = targetWidth / targetHeight;
  double range_inches_per_count = 1.0 / 19.744;
  // double range_inches_per_count = 1.0;
  double min_range = 35;
  double max_range = 70;
  boolean range_error = false;
  double range = 0;

  boolean getRangeError() {
    //double range = range_inches_per_count * rangefinder.getValue();
    double range = rangefinder.getValue();
    if (range > max_range || range < min_range)
      range_error = true;
    else
      range_error = false;
    return range_error;
  }

  public double getRange() {
    if (useSerial) {

      String response;

      try {
        response = ultrasonicPort.readString();
        if (response.isBlank())
          System.out.println("Serial =empty");
        else {
          System.out.println("Serial =(" + response + ")");
          String numberPart = response.substring(1, 5); // remove R character and carriage return
          int distance = Integer.parseInt(numberPart);
          range = (double) distance;
          System.out.println("range=" + range);
        }
      } catch (Exception ex) {
        System.out.println("Get range error: " + ex.toString());
        // ex.printStackTrace();
      }
      return range;
    }
    else {
      double range = range_inches_per_count * rangefinder.getValue();
      return range;
    }
  }

  public void init() {
    rangefinder.setAccumulatorDeadband(0);
    rangefinder.setAccumulatorInitialValue(0);
    rangefinder.resetAccumulator();
    rangefinder.setAccumulatorCenter(2048);
    rangefinder.setAverageBits(2);
    camera = CameraServer.getInstance().startAutomaticCapture("Targeting", 0);
    camera.setFPS(60);
    camera.setResolution((int) imageWidth, (int) imageHeight);
    SmartDashboard.putNumber("Targets", 0);
    // SmartDashboard.putNumber("H distance", 0);
    // SmartDashboard.putNumber("W distance", 1);
    SmartDashboard.putNumber("V offset", 0);
    SmartDashboard.putNumber("H offset", 0);
    SmartDashboard.putNumber("Aspect Ratio", 0);
    SmartDashboard.putNumber("Process Time", 0);
    SmartDashboard.putNumber("Target Width", 0);
    SmartDashboard.putNumber("Target Height", 0);
    SmartDashboard.putNumber("Target Aspect", 0);
    SmartDashboard.putNumber("Target Tilt", 0);
    SmartDashboard.putNumber("Range", 0);
    SmartDashboard.putBoolean("error", false);

    // SmartDashboard.putNumber("Angle", 0);
    SmartDashboard.putBoolean("Show HSV", false);
    System.out.println("fov H:" + Math.toDegrees(cameraFovH) + " W:" + Math.toDegrees(cameraFovW));
    System.out.println("Expected Target Aspect ratio:" + round10(targetAspectRatio));

  }

  Point center(Rect r) {
    double cx = r.tl().x + 0.5 * r.width;
    double cy = r.tl().y + 0.5 * r.height;
    return new Point(cx, cy);
  }

  double round10(double x) {
    return Math.round(x * 10 + 0.5) / 10.0;
  }

  public void run() {
    GripPipeline grip = new GripPipeline();
    CvSink cvSink = CameraServer.getInstance().getVideo();
    CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 320, 240);
    Mat mat = new Mat();
    ArrayList<RotatedRect> rects = new ArrayList<RotatedRect>();
    // TODO: use a network tables data structure to pass target params to Robot
    // Program
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("TargetData");
    edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();
    timer.start();
    try {
      rangingPin = new DigitalOutput(0);
      rangingPin.set(true);
      ultrasonicPort = new SerialPort(9600, Port.kOnboard, 8, Parity.kNone, StopBits.kOne);
      ultrasonicPort.setTimeout(2);
      ultrasonicPort.setReadBufferSize(6);
      ultrasonicPort.reset();
    } catch (RuntimeException ex) {
      System.out.println("Failure to open serial port.");
    }
    while (true) {
      try {
        Thread.sleep(20);
      } catch (InterruptedException ex) {
        System.out.println("exception)");
      }
      timer.reset();
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }
      double dt = timer.get() * 1000;
      grip.process(mat);

      Boolean show_hsv = SmartDashboard.getBoolean("Show HSV", false);
      if (show_hsv) {
        Mat hsv = grip.hsvThresholdOutput(); // display HSV image
        hsv.copyTo(mat);
      }
      ArrayList<MatOfPoint> contours = grip.filterContoursOutput();
      rects.clear();
      double max_area = 0;
      RotatedRect biggest = null;
      // find the bounding boxes of all targets
      // public static RotatedRect minAreaRect(MatOfPoint2f points)
      // Imgproc.boxPoints(RotatedRect box, Mat points)
      for (int i = 0; i < contours.size(); i++) {
        MatOfPoint contour = contours.get(i);
        double area = Imgproc.contourArea(contour);
        MatOfPoint2f NewMtx = new MatOfPoint2f(contour.toArray());
        RotatedRect r = Imgproc.minAreaRect(NewMtx);
        if (area > max_area) {
          biggest = r;
          max_area = area;
        }
        rects.add(r);
      }

      // calculate distance to target
      // - using ht
      SmartDashboard.putNumber("Targets", rects.size());
      if (biggest != null) {
        double h = biggest.size.height;
        double w = biggest.size.width;
        double a = biggest.angle;

        if (biggest.size.width > biggest.size.height) {
          h = biggest.size.width;
          w = biggest.size.height;
          a += 90.0;
        }
        double dh = distanceFactorHeight / biggest.size.height;
        double dw = distanceFactorWidth / biggest.size.width;
        Point ctr = biggest.center;
        double hoff = angleFactorWidth * (ctr.x - 0.5 * imageWidth);
        double voff = -angleFactorHeight * (ctr.y - 0.5 * imageHeight); // invert y !

        // SmartDashboard.putNumber("H distance", round10(dh));
        // SmartDashboard.putNumber("W distance", round10(dw));
        SmartDashboard.putNumber("H offset", round10(hoff));
        SmartDashboard.putNumber("V offset", round10(voff));
        SmartDashboard.putNumber("Aspect Ratio", round10((double) (biggest.size.width) / biggest.size.height));
        SmartDashboard.putNumber("Process Time", dt);
        SmartDashboard.putNumber("Target Width", round10(w));
        SmartDashboard.putNumber("Target Height", round10(h));
        SmartDashboard.putNumber("Target Aspect", round10((double) (w) / h));
        SmartDashboard.putNumber("Target Tilt", round10(a));

        SmartDashboard.putBoolean("error", (getRangeError()));

      }
      double range = getRange();
      table.getEntry("Range").setDouble(range);

      SmartDashboard.putNumber("Range", range);

      for (int i = 0; i < rects.size(); i++) {
        RotatedRect r = rects.get(i);
        // Rect b=r.boundingRect();
        // Point tl = b.tl();
        // Point br = b.br();
        // if (r == biggest)
        // Imgproc.rectangle(mat, tl, br, new Scalar(255.0, 255.0, 0.0), 2);
        // else
        // Imgproc.rectangle(mat, tl, br, new Scalar(255.0, 255.0, 255.0), 1);
        Point[] vertices = new Point[4];
        r.points(vertices);
        for (int j = 0; j < 4; j++) {
          if (r == biggest)
            Imgproc.line(mat, vertices[j], vertices[(j + 1) % 4], new Scalar(255, 255, 0), 2);
          else
            Imgproc.line(mat, vertices[j], vertices[(j + 1) % 4], new Scalar(255, 255, 255), 1);
        }
      }
      outputStream.putFrame(mat);

    }
  }
}
