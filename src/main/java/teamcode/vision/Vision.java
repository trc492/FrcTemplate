/*
 * Copyright (c) 2026 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.vision;

import java.util.Comparator;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcDashboard;
import frclib.robotcore.FrcField;
import frclib.vision.FrcPhotonVision;
import frclib.vision.FrcPhotonVision.DetectedObject;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcVision;
import trclib.vision.TrcVision.CameraInfo;

/**
 * This class is a thin wrapper extending FrcPhotonVision that provides additional game specific functionalities.
 */
public class Vision
{
    private static final String moduleName = Vision.class.getSimpleName();

    private static final String DBKEY_PREFIX            = moduleName + "/";
    private static final String DBKEY_CAM1_PIPELINE     = DBKEY_PREFIX + "Cam1Pipeline";    //Choices
    private static final String DBKEY_CAM2_PIPELINE     = DBKEY_PREFIX + "Cam2Pipeline";    //Choices
    public static final String DBKEY_RELOCALIZE         = DBKEY_PREFIX + "Relocalize";      //Boolean
    private static final String DBKEY_SHOW_STATUS       = DBKEY_PREFIX + "ShowStatus";      //Boolean
    private static final String DBKEY_CAM1              = DBKEY_PREFIX + "Cam1";            //String
    private static final String DBKEY_CAM2              = DBKEY_PREFIX + "Cam2";            //String

    // Microsoft HD-3000 info.
    public static final TrcVision.CameraInfo hd3000CamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("HD-3000", 1280, 720)
        .setCameraPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    // Camera 1 info.
    public static final TrcVision.CameraInfo cam1Info = new TrcVision.CameraInfo()
        .setCameraInfo("Cam1OV9782", 1280, 800)
        .setCameraPose(-0.25, 5.75, 7.0, 0.0, 21.8346, 0.0);
    // Camera 2 info.
    public static final TrcVision.CameraInfo cam2Info = new TrcVision.CameraInfo()
        .setCameraInfo("Cam2OV9782", 1280, 800)
        .setCameraPose(0.0, -1.563, 41.374, 180.0, 9.1241, 0.0);

    public enum PipelineType
    {
        AprilTag(0),
        RedBlob(1),
        BlueBlob(2);

        public int pipelineIndex;

        PipelineType(int value)
        {
            pipelineIndex = value;
        }

    }   //enum PipelineType

    private final TrcDbgTrace tracer;
    private final FrcDashboard dashboard;
    private final Robot robot;
    private final FrcChoiceMenu<PipelineType> cam1PipelineMenu;
    private final FrcChoiceMenu<PipelineType> cam2PipelineMenu;

    public final FrcPhotonVision cam1Vision;
    public final FrcPhotonVision cam2Vision;
    // private final Transform3d cam1FromRobot;
    // private final Transform3d cam2FromRobot;
    private PipelineType cam1Pipeline;
    private PipelineType cam2Pipeline;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for accessing hardware.
     */
    public Vision(Robot robot)
    {
        this.tracer = new TrcDbgTrace();
        this.dashboard = FrcDashboard.getInstance();
        this.robot = robot;

        cam1PipelineMenu = new FrcChoiceMenu<>(DBKEY_CAM1_PIPELINE);
        cam1PipelineMenu.addChoice(PipelineType.AprilTag.name(), PipelineType.AprilTag, true, false);
        cam1PipelineMenu.addChoice(PipelineType.RedBlob.name(), PipelineType.RedBlob);
        cam1PipelineMenu.addChoice(PipelineType.BlueBlob.name(), PipelineType.BlueBlob, false, true);

        cam2PipelineMenu = new FrcChoiceMenu<>(DBKEY_CAM2_PIPELINE);
        cam2PipelineMenu.addChoice(PipelineType.AprilTag.name(), PipelineType.AprilTag, true, false);
        cam2PipelineMenu.addChoice(PipelineType.RedBlob.name(), PipelineType.RedBlob);
        cam2PipelineMenu.addChoice(PipelineType.BlueBlob.name(), PipelineType.BlueBlob, false, true);

        dashboard.refreshKey(DBKEY_RELOCALIZE, RobotParams.Preferences.visionRelocalizeEnabled);
        dashboard.refreshKey(DBKEY_SHOW_STATUS, false);
        dashboard.refreshKey(DBKEY_CAM1, "");
        dashboard.refreshKey(DBKEY_CAM2, "");

        if (robot.robotInfo.camInfos.length > 0 && robot.robotInfo.camInfos[0] != null)
        {
            tracer.traceInfo(
                moduleName, "Creating cam1Vision for camera %s.", robot.robotInfo.camInfos[0].camName);
            cam1Vision = new FrcPhotonVision(
                robot.robotInfo.camInfos[0], this::getAprilTagGroundOffset, this::getRobotToCamera1);
            // cam1FromRobot = new Transform3d(
            //     new Translation3d(Units.inchesToMeters(robot.robotInfo.camInfos[0].camPose.y),
            //                       -Units.inchesToMeters(robot.robotInfo.camInfos[0].camPose.x),
            //                       Units.inchesToMeters(robot.robotInfo.camInfos[0].camPose.z)),
            //     new Rotation3d(Units.degreesToRadians(robot.robotInfo.camInfos[0].camPose.roll),
            //                    -Units.degreesToRadians(robot.robotInfo.camInfos[0].camPose.pitch),
            //                    -Units.degreesToRadians(robot.robotInfo.camInfos[0].camPose.yaw)));
            setCam1Pipeline();
            robot.globalTracer.traceInfo(moduleName, "Setting Cam1 pipeline to " + cam1Pipeline);
        }
        else
        {
            cam1Vision = null;
            // cam1FromRobot = null;
        }

        if (robot.robotInfo.camInfos.length > 1 && robot.robotInfo.camInfos[1] != null)
        {
            tracer.traceInfo(
                moduleName, "Creating cam2Vision for camera %s.", robot.robotInfo.camInfos[1].camName);
            cam2Vision = new FrcPhotonVision(
                robot.robotInfo.camInfos[1], this::getAprilTagGroundOffset, this::getRobotToCamera2);
            // cam2FromRobot = new Transform3d(
            //     new Translation3d(Units.inchesToMeters(robot.robotInfo.camInfos[1].camPose.y),
            //                       -Units.inchesToMeters(robot.robotInfo.camInfos[1].camPose.x),
            //                       Units.inchesToMeters(robot.robotInfo.camInfos[1].camPose.z)),
            //     new Rotation3d(Units.degreesToRadians(robot.robotInfo.camInfos[1].camPose.roll),
            //                    -Units.degreesToRadians(robot.robotInfo.camInfos[1].camPose.pitch),
            //                    -Units.degreesToRadians(robot.robotInfo.camInfos[1].camPose.yaw)));
            setCam2Pipeline();
            robot.globalTracer.traceInfo(moduleName, "Setting Cam2 pipeline to " + cam2Pipeline);
        }
        else
        {
            cam2Vision = null;
            // cam2FromRobot = null;
        }

        FrcDashboard.getInstance().addStatusUpdate(moduleName, this::updateStatus);
    }   //Vision

    /**
     * This method sets camera 1 to the specified pipeline type.
     *
     * @param pipelineType specifies the pipeline to activate in PhotonVision.
     */
    public void setCam1Pipeline(PipelineType pipelineType)
    {
        if (cam1Vision != null && pipelineType != cam1Pipeline)
        {
            cam1Pipeline = pipelineType;
            cam1Vision.setPipelineIndex(pipelineType.pipelineIndex);
        }
    }   //setCam1Pipeline

    /**
     * This method sets camera 1 to the specified pipeline type from the Dashboard.
     *
     * @param pipelineType specifies the pipeline to activate in PhotonVision.
     */
    public void setCam1Pipeline()
    {
        setCam1Pipeline(cam1PipelineMenu.getCurrentChoiceObject());
    }   //setCam1Pipeline

    /**
     * This method sets camera 2 to the specified pipeline type.
     *
     * @param pipelineType specifies the pipeline to activate in PhotonVision.
     */
    public void setCam2Pipeline(PipelineType pipelineType)
    {
        if (cam2Vision != null && pipelineType != cam2Pipeline)
        {
            cam2Pipeline = pipelineType;
            cam2Vision.setPipelineIndex(pipelineType.pipelineIndex);
        }
    }   //setCam2Pipeline

    /**
     * This method sets camera 2 to the specified pipeline type from the Dashboard.
     *
     * @param pipelineType specifies the pipeline to activate in PhotonVision.
     */
    public void setCam2Pipeline()
    {
        setCam2Pipeline(cam2PipelineMenu.getCurrentChoiceObject());
    }   //setCam2Pipeline

    /**
     * This method returns the active camera 1 pipeline.
     *
     * @return active pipeline.
     */
    public PipelineType getCam1Pipeline()
    {
        cam1Pipeline = cam1Vision != null? PipelineType.values()[cam1Vision.getPipelineIndex()]: null;
        return cam1Pipeline;
    }   //getCam1Pipeline

    /**
     * This method returns the active camera 2 pipeline.
     *
     * @return active pipeline.
     */
    public PipelineType getCam2Pipeline()
    {
        cam2Pipeline = cam2Vision != null? PipelineType.values()[cam2Vision.getPipelineIndex()]: null;
        return cam2Pipeline;
    }   //getCam2Pipeline

    /**
     * This method returns the camera position relative to robot center.
     *
     * @return robot to camera transform.
     */
    private Transform3d getRobotToCamera(CameraInfo camInfo)
    {
        if (camInfo != null)
        {
            return new Transform3d(
                new Translation3d(Units.inchesToMeters(camInfo.camPose.y),
                                  -Units.inchesToMeters(camInfo.camPose.x),
                                  Units.inchesToMeters(camInfo.camPose.z)),
                new Rotation3d(Units.degreesToRadians(camInfo.camPose.roll),
                               -Units.degreesToRadians(camInfo.camPose.pitch),
                               -Units.degreesToRadians(camInfo.camPose.yaw)));
        }

        return null;
    }   //getRobotToCamera

    /**
     * This method returns camera 1 position relative to robot center.
     *
     * @return robot to camera transform.
     */
    private Transform3d getRobotToCamera1()
    {
        return getRobotToCamera(cam1Info);
    }   //getRobotToCamera1

    /**
     * This method returns camera 2 position relative to robot center.
     *
     * @return robot to camera transform.
     */
    private Transform3d getRobotToCamera2()
    {
        return getRobotToCamera(cam2Info);
    }   //getRobotToCamera2

    /**
     * This method returns the ground offset of the detected AprilTag.
     *
     * @param object specifes the detected object.
     * @return target ground offset.
     */
    private double getAprilTagGroundOffset(Object object)
    {
        double aprilTagGroundOffset = 0.0;
        Pose3d aprilTagPose = FrcField.getAprilTagFieldPose3d(((PhotonTrackedTarget) object).getFiducialId());
        // Even though PhotonVision said detected target, FieldLayout may not give us AprilTagPose.
        // Check it before access the AprilTag pose.
        if (aprilTagPose != null)
        {
            aprilTagGroundOffset = aprilTagPose.getZ();
        }

        return aprilTagGroundOffset;
    }   //getAprilTagGroundOffset

    /**
     * This method returns the best detected object.
     *
     * @param comparator specifies comparator for sorting the detected AprilTags, can be null if not provided.
     * @param aprilTagIds specifies the AprilTag IDs to look for, can be null if detecting any AprilTags.
     * @return best detected AprilTag.
     */
    public DetectedObject getBestDetectedAprilTag(
        Comparator<? super PhotonTrackedTarget> comparator, int... aprilTagIds)
    {
        // Camera 1 is used for detecting AprilTag.
        DetectedObject detectedAprilTag = cam1Vision != null && cam1Pipeline == PipelineType.AprilTag?
            cam1Vision.getDetectedAprilTag(comparator, aprilTagIds): null;

        if (detectedAprilTag != null)
        {
            tracer.traceDebug(moduleName, "AprilTag=%d", detectedAprilTag.target.getFiducialId());
            if  (robot.ledIndicator != null)
            {
                // Show result using LED.
                robot.ledIndicator.setPhotonDetectedObject(PipelineType.AprilTag, detectedAprilTag);
            }
        }

        return detectedAprilTag;
    }   //getBestDetectedAprilTag

    /**
     * This method returns the best detected color blob.
     *
     * @param pipelineType specifies the pipeline type for color blob detection.
     * @param comparator specifies comparator for sorting the detected objects, can be null if not provided.
     * @return best detected object.
     */
    public DetectedObject getBestDetectedColorBlob(
        PipelineType pipelineType, Comparator<? super PhotonTrackedTarget> comparator)
    {
        // Camera 2 is used for detecting color blobs.
        DetectedObject detectedObj = cam2Vision != null && cam2Pipeline == pipelineType?
            cam2Vision.getBestDetectedObject(comparator): null;

        if (detectedObj != null && robot.ledIndicator != null)
        {
            robot.ledIndicator.setPhotonDetectedObject(pipelineType, detectedObj);
        }

        return detectedObj;
    }   //getBestDetectedColorBlob

    /**
     * This method is called by the comparator to sort the detected object array in descending area of the target.
     *
     * @param t1 specifies the target 1 object.
     * @param t2 specifies the target 2 object.
     * @return positive value if target 2 area is greater than target 1, negative value if target 2 area is smaller
     *         than target 1, zero if areas are equal.
     */
    public int compareAreas(PhotonTrackedTarget t1, PhotonTrackedTarget t2)
    {
        return (int)((t2.getArea() - t1.getArea())*100);
    }   //compareArea

    // /**
    //  * This method determines the robot's absolute field pose by averaging the robotFieldPose determined by both
    //  * camera 1 and 2.
    //  *
    //  * @return averaged robot field pose.
    //  */
    // public TrcPose2D getRobotFieldPose()
    // {
    //     TrcPose2D robotFieldPoseFromCam1 =
    //         cam1Vision != null? cam1Vision.getRobotEstimatedPose(cam1FromRobot): null;
    //     TrcPose2D robotFieldPoseFromCam2 =
    //         cam2Vision != null? cam2Vision.getRobotEstimatedPose(cam2FromRobot): null;
    //     // Average the robotFieldPose from both cameras.
    //     TrcPose2D robotFieldPose = new TrcPose2D(
    //         (robotFieldPoseFromCam1.x + robotFieldPoseFromCam2.x)/2.0,
    //         (robotFieldPoseFromCam1.y + robotFieldPoseFromCam2.y)/2.0,
    //         (robotFieldPoseFromCam1.angle + robotFieldPoseFromCam2.angle)/2.0);

    //     TrcDbgTrace.globalTraceDebug(
    //         moduleName, "RobotPoseCam1=%s, RobotPoseCam2=%s, RobotPose=%s",
    //         robotFieldPoseFromCam1, robotFieldPoseFromCam2, robotFieldPose);
    //     return robotFieldPose;
    // }   //getRobotFieldPose

    /**
     * This method determines the closest AprilTag from the given robot pose.
     *
     * @param robotPose specifies the robot pose.
     * @return closest AprilTag pose.
     */
    public static TrcPose2D getClosestAprilTagPose(TrcPose2D robotPose)
    {
        TrcPose2D closestAprilTagPose = null;
        double minDistance = Double.MAX_VALUE;

        for (TrcPose2D aprilTagPose: RobotParams.Game.aprilTagFieldPoses)
        {
            double distance = robotPose.distanceTo(aprilTagPose);

            if (distance < minDistance)
            {
                minDistance = distance;
                closestAprilTagPose = aprilTagPose;
            }
        }

        return closestAprilTagPose.clone();
    }   //getClosestAprilTagPose

    // private final FrcUserChoices visionChoices = new FrcUserChoices();

    // /**
    //  * This method publishes the NetworkTable entries for the subsystem to the Dashboard.
    //  */
    // private void publishToDashboard()
    // {
    //     visionChoices.addChoiceMenu(DBKEY_CAM1_PIPELINES, cam1PipelineMenu);
    //     visionChoices.addChoiceMenu(DBKEY_CAM2_PIPELINES, cam1PipelineMenu);

    // }   //publishToDashboard

    /**
     * This method update the dashboard with vision status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (slowLoop && dashboard.getBoolean(DBKEY_SHOW_STATUS, false))
        {
            DetectedObject detectedObj;

            if (cam1Vision != null)
            {
                if (cam1Pipeline == PipelineType.AprilTag)
                {
                    detectedObj = cam1Vision.getDetectedAprilTag(null, null);
                    if (detectedObj != null)
                    {
                        String msg = String.format(
                            "Cam1Vision[%d]:targetPose=%s,robotPose=%s",
                            detectedObj.target.getFiducialId(), detectedObj.targetPose, detectedObj.robotPose);
                        dashboard.putString(DBKEY_PREFIX + "Cam1", msg);
                        // dashboard.displayPrintf(lineNum++, msg);
                    }
                    // else
                    // {
                    //     lineNum++;
                    // }
                }
                else
                {
                    detectedObj = cam1Vision.getBestDetectedObject(this::compareAreas);
                    if (detectedObj != null)
                    {
                        String msg = String.format(
                            "Cam1Vision(%s): targetPose=%s, robotPose=%s",
                            cam1Pipeline, detectedObj.targetPose, detectedObj.robotPose);
                        dashboard.putString(DBKEY_PREFIX + "Cam1", msg);
                        // dashboard.displayPrintf(lineNum++, msg);
                    }
                    // else
                    // {
                    //     lineNum++;
                    // }
                }
            }

            if (cam2Vision != null)
            {
                if (cam2Pipeline == PipelineType.AprilTag)
                {
                    detectedObj = cam2Vision.getDetectedAprilTag(null, null);
                    if (detectedObj != null)
                    {
                        String msg = String.format(
                            "Cam2Vision[%d]:targetPose=%s,robotPose=%s",
                            detectedObj.target.getFiducialId(), detectedObj.targetPose, detectedObj.robotPose);
                        dashboard.putString(DBKEY_PREFIX + "Cam2", msg);
                        // dashboard.displayPrintf(lineNum++, msg);
                    }
                    // else
                    // {
                    //     lineNum++;
                    // }
                }
                else
                {
                    detectedObj = cam2Vision.getBestDetectedObject(this::compareAreas);
                    if (detectedObj != null)
                    {
                        String msg = String.format(
                            "Cam2Vision(%s): targetPose=%s, robotPose=%s",
                            cam2Pipeline, detectedObj.targetPose, detectedObj.robotPose);
                        dashboard.putString(DBKEY_PREFIX + "Cam2", msg);
                        // dashboard.displayPrintf(lineNum++, msg);
                    }
                    // else
                    // {
                    //     lineNum++;
                    // }
                }
            }
        }

        return lineNum;
    }   //updateStatus

}   //class Vision
