// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.io.File;
import java.io.FileFilter;
import java.io.FilenameFilter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
       "switched cameras": [
           {
               "name": <virtual camera name>
               "key": <network table key used for selection>
               // if NT value is a string, it's treated as a name
               // if NT value is a double, it's treated as an integer index
           }
       ]
   }
 */

public final class Main {
  public static final String Constants = null;
  private static String configFile = "/boot/frc.json";

  @SuppressWarnings("MemberName")
  public static class CameraConfig {
    public String name;
    public String path;
    public JsonObject config;
    public JsonElement streamConfig;
  }

  @SuppressWarnings("MemberName")
  public static class SwitchedCameraConfig {
    public String name;
    public String key;
  };

  public static int team;
  public static boolean server;
  public static List<CameraConfig> cameraConfigs = new ArrayList<>();
  public static List<SwitchedCameraConfig> switchedCameraConfigs = new ArrayList<>();
  public static List<VideoSource> cameras = new ArrayList<>();
  public static CvSource output;

  private Main() {
  }

  /**
   * Report parse error.
   */
  public static void parseError(String str) {
    System.err.println("config error in '" + configFile + "': " + str);
  }

  /**
   * Read single camera configuration.
   */
  public static boolean readCameraConfig(JsonObject config) {
    CameraConfig cam = new CameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement pathElement = config.get("path");
    if (pathElement == null) {
      parseError("camera '" + cam.name + "': could not read path");
      return false;
    }
    cam.path = pathElement.getAsString();

    // stream properties
    cam.streamConfig = config.get("stream");

    cam.config = config;

    cameraConfigs.add(cam);
    return true;
  }

  /**
   * Read single switched camera configuration.
   */
  public static boolean readSwitchedCameraConfig(JsonObject config) {
    SwitchedCameraConfig cam = new SwitchedCameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read switched camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement keyElement = config.get("key");
    if (keyElement == null) {
      parseError("switched camera '" + cam.name + "': could not read key");
      return false;
    }
    cam.key = keyElement.getAsString();

    switchedCameraConfigs.add(cam);
    return true;
  }

  /**
   * Read configuration file.
   */
  @SuppressWarnings("PMD.CyclomaticComplexity")
  public static boolean readConfig() {
    // parse file
    JsonElement top;
    try {
      top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
    } catch (IOException ex) {
      System.err.println("could not open '" + configFile + "': " + ex);
      return false;
    }

    // top level must be an object
    if (!top.isJsonObject()) {
      parseError("must be JSON object");
      return false;
    }
    JsonObject obj = top.getAsJsonObject();

    // team number
    JsonElement teamElement = obj.get("team");
    if (teamElement == null) {
      parseError("could not read team number");
      return false;
    }
    team = teamElement.getAsInt();

    // ntmode (optional)
    if (obj.has("ntmode")) {
      String str = obj.get("ntmode").getAsString();
      if ("client".equalsIgnoreCase(str)) {
        server = false;
      } else if ("server".equalsIgnoreCase(str)) {
        server = true;
      } else {
        parseError("could not understand ntmode value '" + str + "'");
      }
    }

    // cameras
    JsonElement camerasElement = obj.get("cameras");
    if (camerasElement == null) {
      parseError("could not read cameras");
      return false;
    }
    JsonArray cameras = camerasElement.getAsJsonArray();
    for (JsonElement camera : cameras) {
      if (!readCameraConfig(camera.getAsJsonObject())) {
        return false;
      }
    }

    if (obj.has("switched cameras")) {
      JsonArray switchedCameras = obj.get("switched cameras").getAsJsonArray();
      for (JsonElement camera : switchedCameras) {
        if (!readSwitchedCameraConfig(camera.getAsJsonObject())) {
          return false;
        }
      }
    }

    return true;
  }

  /**
   * Start running the camera.
   */
  public static VideoSource startCamera(CameraConfig config) {
    System.out.println("Starting camera '" + config.name + "' on " + config.path);
    CameraServer inst = CameraServer.getInstance();
    UsbCamera camera = new UsbCamera(config.name, config.path);
    camera.setFPS(20);
    MjpegServer server = inst.startAutomaticCapture(camera);

    output = inst.putVideo("CameraHub-Output", 320, 240);
    output.setVideoMode(PixelFormat.kGray, 320, 240, 6);

    Gson gson = new GsonBuilder().create();

    camera.setConfigJson(gson.toJson(config.config));
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    if (config.streamConfig != null) {
      server.setConfigJson(gson.toJson(config.streamConfig));
    }

    camera.setFPS(20);
    output.setFPS(6);

    return camera;
  }

  /**
   * Start running the switched camera.
   */
  public static MjpegServer startSwitchedCamera(SwitchedCameraConfig config) {
    System.out.println("Starting switched camera '" + config.name + "' on " + config.key);
    MjpegServer server = CameraServer.getInstance().addSwitchedCamera(config.name);

    NetworkTableInstance.getDefault()
        .getEntry(config.key)
        .addListener(event -> {
              if (event.value.isDouble()) {
                int i = (int) event.value.getDouble();
                if (i >= 0 && i < cameras.size()) {
                  server.setSource(cameras.get(i));
                }
              } else if (event.value.isString()) {
                String str = event.value.getString();
                for (int i = 0; i < cameraConfigs.size(); i++) {
                  if (str.equals(cameraConfigs.get(i).name)) {
                    server.setSource(cameras.get(i));
                    break;
                  }
                }
              }
            },
            EntryListenerFlags.kImmediate | EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    return server;
  }

  /**
   * Example pipeline.
   */
  public static class MyPipeline implements VisionPipeline {
    private static final int SAMPLE_SIZE = 15;
    public HubGripPipeline hubGrip = new HubGripPipeline();
    public static File[] listOfFiles;
    public static int counter = 0;
    private static double focalLength;

    MedianFilter minYFilter = new MedianFilter(SAMPLE_SIZE);
    MedianFilter maxYFilter = new MedianFilter(SAMPLE_SIZE);
    MedianFilter minXFilter = new MedianFilter(SAMPLE_SIZE);
    MedianFilter maxXFilter = new MedianFilter(SAMPLE_SIZE);
    MedianFilter hubDistanceFilter = new MedianFilter(SAMPLE_SIZE);

    // static {
    //   File folder = new File("/home/pi/2022VisionSampleImages/");
    //   FilenameFilter filter = (file, s) -> s.toLowerCase().endsWith(".png");
    //   listOfFiles = folder.listFiles(filter);
    //   // System.out.println("List of Files length: " + listOfFiles.length);
    // }

    @Override
    public void process(Mat mat) {
      // System.out.println("List of Files length: " + listOfFiles.length);
      
      // File file = listOfFiles[counter++];
      // File file = new File("/home/pi/2022VisionSampleImages/TarmacCenter7ft10in.png");
      // Mat newFrame = Imgcodecs.imread(file.getAbsolutePath());
      // Mat newFrame = Imgcodecs.imread("/home/pi/2022VisionSampleImages/TarmacCenter2ft10in.png");
      Mat newFrame = mat;

      // if (counter >= listOfFiles.length)
        // counter = 0;
      
      hubGrip.process(newFrame);
      // hubGrip.process(mat);
      // Point crossHair = findBoundingBoxesHub(mat);
      Point[] hubBounds = findBoundingBoxesHub(newFrame, hubGrip.filterContoursOutput());

      if (hubBounds != null){
        RotatedRect rect = findMidRect(hubGrip.filterContoursOutput());

        double width = hubDistanceFilter.calculate(Double.valueOf(rect.boundingRect().width));
        double finalDistance = findDistance((int) width);
          
        NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
        NetworkTableEntry teDist = ntinst.getTable("Vision").getEntry("hubDistance");
        teDist.setDouble(finalDistance);

        drawRect(hubBounds, newFrame);
        Point ch = findCrosshair(hubBounds);
        
        NetworkTableEntry teAngle = ntinst.getTable("Vision").getEntry("hubAngle");
        double hubAngle = 600; // 600 is angle not found
        
        if (ch != null){
          drawCrosshair(ch, newFrame);        
          hubAngle = calculateAngle(ch);
          teAngle.setDouble(hubAngle); 
        }
        
        int location = 0;
        // Imgproc.putText(newFrame, file.getName(), new Point(0, location = location + 25), Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 255), 2); 
        Imgproc.putText(newFrame, String.format("Angle: %.2f Deg", hubAngle), new Point(0, location = location + 30), Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 255), 2); 
        Imgproc.putText(newFrame, String.format("Dist: %.2f\"", finalDistance), new Point(newFrame.width()-125, 25), Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 0, 255), 2); 
      }
      else {
        Imgproc.putText(newFrame, "Target Not Detected", new Point(0, 30), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 0, 255), 2); 
      }
      output.putFrame(newFrame);
    }

    public RotatedRect findMidRect(ArrayList<MatOfPoint> contours) {
      RotatedRect[] rects = findBoundingBoxes(contours);

      if (rects.length >= 2)
          return rects[1];
      else
          return rects[0];
    }

    public RotatedRect[] findBoundingBoxes(ArrayList<MatOfPoint> contours) {
      // System.out.println(contours.size());
      RotatedRect[] rects = new RotatedRect[contours.size()];
      for (int i = 0; i < contours.size(); i++)
          rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));

      return rects;
    }

    public double calculateAngle(Point crosshair) {
      int pixelDistance = (int) crosshair.x - (320/2);
      double angle = pixelDistance * 0.1875; //Horizontal degrees per pixel for this camera is 0.1875
      return angle;
    }

    // Draw the crosshair on the frame
    public void drawCrosshair(Point crosshair, Mat mat, Scalar... color) {
      Scalar chosenColor = new Scalar(255, 255, 255);

      if (color.length >= 1){
        chosenColor = color[0];
      }

      Imgproc.line(mat, new Point(crosshair.x - 5, crosshair.y - 5), new Point(crosshair.x + 5, crosshair.y + 5), chosenColor, 3);
      Imgproc.line(mat, new Point(crosshair.x - 5, crosshair.y + 5), new Point(crosshair.x + 5, crosshair.y - 5), chosenColor, 3);
    }

    // Draw bounding box around the reflective tape in Green
    public void drawRect(Point[] pts, Mat mat, Scalar... color) {
      Scalar chosenColor = new Scalar(0, 255, 0);
      if (color.length >= 1){
        chosenColor = color[0];
      }
      for (int i = 0; i < 4; i++)
          Imgproc.line(mat, pts[i], pts[(i + 1) % 4], chosenColor, 2);
    }

    // Calculate the crosshair position
    public Point findCrosshair(Point[] pts) {
      // i is starting point for line, j is next point
      //int j;
      Point crosshair = new Point((pts[0].x + pts[2].x) / 2, (pts[0].y + pts[2].y) / 2);
      /*
      for (int i = 0; i < 4; i++) {
          j = (i + 1) % 4;
          if (crosshair == null || (pts[i].y + pts[j].y) / 2 < crosshair.y)
              crosshair = new Point((pts[i].x + pts[j].x) / 2, (pts[i].y + pts[j].y) / 2);
      }
      */
      return crosshair;
    }

    public Point[] findBoundingBoxesHub(Mat hubMat, ArrayList<MatOfPoint> contours) {
      // ArrayList<MatOfPoint> contours = hubGrip.filterContoursOutput();
      System.out.println("Contours Size: " + contours.size());
      RotatedRect[] rects = new RotatedRect[contours.size()];
      for (int i = 0; i < contours.size(); i++)
          rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));

      Point hubPts[] = null;

      if (contours.size() != 0){
          double minX = rects[0].boundingRect().x;
          double maxX = 0;
          double minY = rects[0].boundingRect().y;
          double maxY = 0;
          
         

          for(int a=0; a<rects.length; a++ ) {
              Point[] ppts = new Point[4];
              rects[a].points(ppts);
              drawRect(ppts, hubMat, new Scalar(0, 0, 255));
              minX = Math.min(minX, rects[a].boundingRect().x);
              maxX = Math.max(maxX, rects[a].boundingRect().x + rects[a].boundingRect().width);
              minY = Math.min(minY, rects[a].boundingRect().y);
              maxY = Math.max(maxY, rects[a].boundingRect().y + rects[a].boundingRect().height);
          }

          double fMinY = minYFilter.calculate(minY);
          double fMaxY = maxYFilter.calculate(maxY);
          double fMinX = minXFilter.calculate(minX);
          double fMaxX = maxXFilter.calculate(maxX);

          // System.out.println(String.format("Min=(%f,%f) Max=(%f,%f)", minX, maxX, minY, maxY));
          // System.out.println(String.format("fMin=(%f,%f) fMax=(%f,%f)", fMinX, fMaxX, fMinY, fMaxY));
        
        //RotatedRect boundingBox = new RotatedRect();
    
        hubPts = new Point[4];
        hubPts[0] = new Point(fMinX, fMinY);
        hubPts[1] = new Point(fMaxX, fMinY);
        hubPts[2] = new Point(fMaxX, fMaxY);
        hubPts[3] = new Point(fMinX, fMaxY);
      }

      return hubPts;
    }

    private double findDistance(int pixelWidth) {
      int calculatedDistance = 0; 
      
      double width = 1016; // width of the hub in mm
      // double fieldOfView = 68.5;
      //focalLength = (pixels * initialDistance) / width;
      // double radVal = Math.toRadians(fieldOfView);
      // double arcTanVal = 240 / 320;
      // double cosVal = Math.cos(arcTanVal);
      // double tanVal = Math.tan(radVal * cosVal);
      // double angrad = Math.atan(tanVal);
      // double horizontalFieldOfView = Math.toDegrees(angrad);
      // // H_FOV = np.degrees(np.arctan(np.tan(np.radians(D_FOV)*np.cos(np.arctan(height/width)))))
      // double focalLength = 320/ (2*Math.tan(Math.toRadians(horizontalFieldOfView/2)));
      double focalLength = findFocalLength();
      // System.out.println("Focal Length: " + focalLength);

      //focalLength = 60; // mm https://commons.wikimedia.org/wiki/File:Microsoft_Lifecam_HD-3000_webcam.jpg
      calculatedDistance = (int) (((width * focalLength) / pixelWidth)/25.4); // in inch

      // // equation for accurate distance to hub
      // //double finalError = error + slope * (calculatedDistance - initialDistance);
      // // =8+0.125*(A10-96)
      // double a = 0.00199916;
      // double b = -0.450253;
      // double c = 35.7879;

      // // double parabolaError = a * Math.pow(calculatedDistance, 2) + b * calculatedDistance + c;
      // double parabolaError = a * (calculatedDistance * calculatedDistance) + (b * calculatedDistance) + c;
      //0.00199916x^{2}-0.450253x+35.7879

      double finalDistance = calculatedDistance;
      return finalDistance;
    }

    public static double findFocalLength(){
        
      if (focalLength == 0){
          // Adjust field of view for the camera type - this is for Microsoft Lifecam HD-3000
          double fieldOfView = 68.5; //Source: https://dl2jx7zfbtwvr.cloudfront.net/specsheets/WEBC1010.pdf
          double radVal = Math.toRadians(fieldOfView);
          double arcTanVal = Math.atan((double) 240 / (double) 320);
          double cosVal = Math.cos(arcTanVal);
          double tanVal = Math.tan(radVal * cosVal);
          double angrad = Math.atan(tanVal);
          double horizontalFieldOfView = Math.toDegrees(angrad);
          // H_FOV = np.degrees(np.arctan(np.tan(np.radians(D_FOV)*np.cos(np.arctan(height/width)))))

          // focal Length f = A / tan(a) where A = frame width / 2 and a = HFOV / 2 in radians
          focalLength = 320/(2*Math.tan(Math.toRadians(horizontalFieldOfView/2)));
      }

      return focalLength;
  }
  }

  /**
   * Main.
   */
  public static void main(String... args) {
    if (args.length > 0) {
      configFile = args[0];
    }

    // read configuration
    if (!readConfig()) {
      return;
    }

    // start NetworkTables
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntinst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntinst.startClientTeam(team);
      ntinst.startDSClient();
    }

    // start cameras
    for (CameraConfig config : cameraConfigs) {
      cameras.add(startCamera(config));
    }
    // cameras.add(output);

    // start switched cameras
    for (SwitchedCameraConfig config : switchedCameraConfigs) {
      startSwitchedCamera(config);
    }

    // start image processing on camera 0 if present
    if (cameras.size() >= 1) {
      VisionThread visionThread = new VisionThread(cameras.get(0),
              new MyPipeline(), pipeline -> {
        // do something with pipeline results
      });
      /* something like this for GRIP:
      VisionThread visionThread = new VisionThread(cameras.get(0),
              new GripPipeline(), pipeline -> {
        ...
      });
       */
      visionThread.start();
    }

    // loop forever
    for (;;) {
      try {
        Thread.sleep(10000);
      } catch (InterruptedException ex) {
        return;
      }
    }
  }
}
