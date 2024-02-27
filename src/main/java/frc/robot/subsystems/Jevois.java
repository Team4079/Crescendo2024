// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

import org.opencv.video.Video;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Jevois extends SubsystemBase {
  /** Creates a new Jevois. */
  private static final int BAUD = 9600;
  private SerialPort serialPort;
  private boolean isConnected;
  private Thread packetListenerThread;
  private UsbCamera camera;


  public Jevois() {
    camera = CameraServer.startAutomaticCapture(0);
    camera.setResolution(320, 240);
    camera.setFPS(30);
    camera.setVideoMode();
    
    CvSink cvSink = CameraServer.getVideo();
    // CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(camera.getInfo());
    SmartDashboard.putBoolean("Jevois Connected", camera.isConnected());
  }
}
