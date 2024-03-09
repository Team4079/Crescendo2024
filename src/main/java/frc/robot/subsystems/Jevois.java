// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The {@link Jevois} class is the camera class which publishes the camera feed to the SmartDashboard.
 * When a camera is detected, the SmartDashboard will display it as connected.
 */
public class Jevois extends SubsystemBase {
    /** Creates a new Jevois. */
    private UsbCamera camera;

    public Jevois() {
        // Start the camera
        camera = CameraServer.startAutomaticCapture(0);
        // Set the resolution and FPS
        camera.setResolution(320, 240);
        camera.setFPS(30);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Jevois Connected", camera.isConnected());
    }
}