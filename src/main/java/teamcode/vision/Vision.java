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
import teamcode.Dashboard;
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
    private final String moduleName = getClass().getSimpleName();

    // Microsoft HD-3000 camera parameters.
    public static final TrcVision.CameraInfo hd3000CamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("HD-3000", 1280, 720)
        .setCameraPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    // Front camera info
    public static final TrcVision.CameraInfo frontCamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("FrontOV9782", 1280, 800)
        .setCameraPose(-0.25, 5.75, 7.0, 0.0, 21.8346, 0.0);
    // Back camera info
    public static final TrcVision.CameraInfo backCamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("BackOV9782", 1280, 800)
        .setCameraPose(0.0, -1.563, 41.374, 180.0, 9.1241, 0.0);

    private static final String DBKEY_PREFIX                = "Vision/";

    public enum PipelineType
    {
        APRILTAG(0),
        RED_BLOB(1),
        BLUE_BLOB(2);

        public int pipelineIndex;

        PipelineType(int value)
        {
            pipelineIndex = value;
        }

    }   //enum PipelineType

    private final TrcDbgTrace tracer;
    private final FrcDashboard dashboard;
    private final Robot robot;
    private final FrcChoiceMenu<PipelineType> frontCamPipelineMenu;
    private final FrcChoiceMenu<PipelineType> backCamPipelineMenu;

    public final FrcPhotonVision frontVision;
    public final FrcPhotonVision backVision;
    private final Transform3d frontCamFromRobot;
    private final Transform3d backCamFromRobot;
    private PipelineType frontCamPipeline;
    private PipelineType backCamPipeline;

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

        frontCamPipelineMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_VISION_FRONTCAM_PIPELINE);
        frontCamPipelineMenu.addChoice("AprilTag", PipelineType.APRILTAG, true, false);
        frontCamPipelineMenu.addChoice("RedBlob", PipelineType.RED_BLOB);
        frontCamPipelineMenu.addChoice("BlueBlob", PipelineType.BLUE_BLOB, false, true);

        backCamPipelineMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_VISION_BACKCAM_PIPELINE);
        backCamPipelineMenu.addChoice("AprilTag", PipelineType.APRILTAG);
        backCamPipelineMenu.addChoice("RedBlob", PipelineType.RED_BLOB, true, false);
        backCamPipelineMenu.addChoice("BlueBlob", PipelineType.BLUE_BLOB, false, true);

        if (robot.robotInfo.camInfos.length > 0 && robot.robotInfo.camInfos[0] != null)
        {
            tracer.traceInfo(
                moduleName, "Creating frontVision for camera %s.", robot.robotInfo.camInfos[0].camName);
            frontVision = new FrcPhotonVision(
                robot.robotInfo.camInfos[0], this::getAprilTagGroundOffset, this::getRobotToFrontCamera);
            frontCamFromRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(robot.robotInfo.camInfos[0].camPose.y),
                                  -Units.inchesToMeters(robot.robotInfo.camInfos[0].camPose.x),
                                  Units.inchesToMeters(robot.robotInfo.camInfos[0].camPose.z)),
                new Rotation3d(Units.degreesToRadians(robot.robotInfo.camInfos[0].camPose.roll),
                               -Units.degreesToRadians(robot.robotInfo.camInfos[0].camPose.pitch),
                               -Units.degreesToRadians(robot.robotInfo.camInfos[0].camPose.yaw)));
            dashboard.refreshKey(DBKEY_PREFIX + robot.robotInfo.camInfos[0].camName, "");
            setFrontCamPipeline();
            robot.globalTracer.traceInfo(moduleName, "Setting FrontCam pipeline to " + frontCamPipeline);
        }
        else
        {
            frontVision = null;
            frontCamFromRobot = null;
        }

        if (robot.robotInfo.camInfos.length > 1 && robot.robotInfo.camInfos[1] != null)
        {
            tracer.traceInfo(
                moduleName, "Creating backVision for camera %s.", robot.robotInfo.camInfos[1].camName);
            backVision = new FrcPhotonVision(
                robot.robotInfo.camInfos[1], this::getAprilTagGroundOffset, this::getRobotToBackCamera);
            backCamFromRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(robot.robotInfo.camInfos[1].camPose.y),
                                  -Units.inchesToMeters(robot.robotInfo.camInfos[1].camPose.x),
                                  Units.inchesToMeters(robot.robotInfo.camInfos[1].camPose.z)),
                new Rotation3d(Units.degreesToRadians(robot.robotInfo.camInfos[1].camPose.roll),
                               -Units.degreesToRadians(robot.robotInfo.camInfos[1].camPose.pitch),
                               -Units.degreesToRadians(robot.robotInfo.camInfos[1].camPose.yaw)));
            dashboard.refreshKey(DBKEY_PREFIX + robot.robotInfo.camInfos[1].camName, "");
            setBackCamPipeline();
            robot.globalTracer.traceInfo(moduleName, "Setting BackCam pipeline to " + backCamPipeline);
        }
        else
        {
            backVision = null;
            backCamFromRobot = null;
        }

        FrcDashboard.getInstance().addStatusUpdate(moduleName, this::updateStatus);
    }   //Vision

    /**
     * This method sets the front camera to the specified pipeline type.
     *
     * @param pipelineType specifies the pipeline to activate in PhotonVision.
     */
    public void setFrontCamPipeline(PipelineType pipelineType)
    {
        if (frontVision != null && pipelineType != frontCamPipeline)
        {
            frontCamPipeline = pipelineType;
            frontVision.setPipelineIndex(pipelineType.pipelineIndex);
        }
    }   //setFrontCamPipeline

    /**
     * This method sets the front camera to the specified pipeline type from the Dashboard.
     *
     * @param pipelineType specifies the pipeline to activate in PhotonVision.
     */
    public void setFrontCamPipeline()
    {
        setFrontCamPipeline(frontCamPipelineMenu.getCurrentChoiceObject());
    }   //setFrontCamPipeline

    /**
     * This method sets the back camera to the specified pipeline type.
     *
     * @param pipelineType specifies the pipeline to activate in PhotonVision.
     */
    public void setBackCamPipeline(PipelineType pipelineType)
    {
        if (backVision != null && pipelineType != backCamPipeline)
        {
            backCamPipeline = pipelineType;
            backVision.setPipelineIndex(pipelineType.pipelineIndex);
        }
    }   //setBackCamPipeline

    /**
     * This method sets the back camera to the specified pipeline type from the Dashboard.
     *
     * @param pipelineType specifies the pipeline to activate in PhotonVision.
     */
    public void setBackCamPipeline()
    {
        setBackCamPipeline(backCamPipelineMenu.getCurrentChoiceObject());
    }   //setBackCamPipeline

    /**
     * This method returns the active front camera pipeline.
     *
     * @return active pipeline.
     */
    public PipelineType getFrontCamPipeline()
    {
        frontCamPipeline = frontVision != null? PipelineType.values()[frontVision.getPipelineIndex()]: null;
        return frontCamPipeline;
    }   //getFrontCamPipeline

    /**
     * This method returns the active back camera pipeline.
     *
     * @return active pipeline.
     */
    public PipelineType getBackCamPipeline()
    {
        backCamPipeline = backVision != null? PipelineType.values()[backVision.getPipelineIndex()]: null;
        return backCamPipeline;
    }   //getBackCamPipeline

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
     * This method returns the front camera position relative to robot center.
     *
     * @return robot to camera transform.
     */
    private Transform3d getRobotToFrontCamera()
    {
        return getRobotToCamera(frontCamInfo);
    }   //getRobotToFrontCamera

    /**
     * This method returns the back camera position relative to robot center.
     *
     * @return robot to camera transform.
     */
    private Transform3d getRobotToBackCamera()
    {
        return getRobotToCamera(backCamInfo);
    }   //getRobotToBackCamera

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
        DetectedObject detectedAprilTag = frontVision != null && frontCamPipeline == PipelineType.APRILTAG?
            frontVision.getDetectedAprilTag(comparator, aprilTagIds): null;

        if (detectedAprilTag != null)
        {
            tracer.traceDebug(moduleName, "AprilTag=%d", detectedAprilTag.target.getFiducialId());
            if  (robot.ledIndicator != null)
            {
                // Show result using LED.
                robot.ledIndicator.setPhotonDetectedObject(PipelineType.APRILTAG, detectedAprilTag);
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
        // Back camera is used for detecting color blobs.
        DetectedObject detectedObj = backVision != null && backCamPipeline == pipelineType?
            backVision.getBestDetectedObject(comparator): null;

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

    /**
     * This method determines the robot's absolute field pose by averaging the robotFieldPose determined by both the
     * left and right shooters.
     *
     * @return averaged robot field pose.
     */
    public TrcPose2D getRobotFieldPose()
    {
        TrcPose2D robotFieldPoseFromFrontCam =
            frontVision != null? frontVision.getRobotEstimatedPose(frontCamFromRobot): null;
        TrcPose2D robotFieldPoseFromBackCam =
            backVision != null? backVision.getRobotEstimatedPose(backCamFromRobot): null;
        // Average the robotFieldPose from the front and back shooter cam.
        TrcPose2D robotFieldPose = new TrcPose2D(
            (robotFieldPoseFromFrontCam.x + robotFieldPoseFromBackCam.x)/2.0,
            (robotFieldPoseFromFrontCam.y + robotFieldPoseFromBackCam.y)/2.0,
            (robotFieldPoseFromFrontCam.angle + robotFieldPoseFromBackCam.angle)/2.0);

        TrcDbgTrace.globalTraceDebug(
            moduleName, "RobotPoseFront=%s, RobotPoseBack=%s, RobotPose=%s",
            robotFieldPoseFromFrontCam, robotFieldPoseFromBackCam, robotFieldPose);
        return robotFieldPose;
    }   //getRobotFieldPose

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

    /**
     * This method update the dashboard with vision status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (slowLoop)
        {
            if (dashboard.getBoolean(
                    Dashboard.DBKEY_PREFERENCE_VISION_STATUS, RobotParams.Preferences.showVisionStatus))
            {
                DetectedObject detectedObj;

                if (frontVision != null && frontCamPipeline == PipelineType.APRILTAG)
                {
                    detectedObj = frontVision.getDetectedAprilTag(null, null);
                    if (detectedObj != null)
                    {
                        String msg = String.format(
                            "FrontVision[%d]:targetPose=%s,robotPose=%s",
                            detectedObj.target.getFiducialId(), detectedObj.targetPose, detectedObj.robotPose);
                        dashboard.putString(DBKEY_PREFIX + "FrontCam", msg);
                        dashboard.displayPrintf(lineNum++, msg);
                    }
                    else
                    {
                        lineNum++;
                    }
                }

                if (backVision != null)
                {
                    if (backCamPipeline == PipelineType.APRILTAG)
                    {
                        detectedObj = backVision.getDetectedAprilTag(null, null);
                        if (detectedObj != null)
                        {
                            String msg = String.format(
                                "BackVision[%d]:targetPose=%s,robotPose=%s",
                                detectedObj.target.getFiducialId(), detectedObj.targetPose, detectedObj.robotPose);
                            dashboard.putString(DBKEY_PREFIX + "BackCam", msg);
                            dashboard.displayPrintf(lineNum++, msg);
                        }
                        else
                        {
                            lineNum++;
                        }
                    }
                    else
                    {
                        detectedObj = backVision.getBestDetectedObject(this::compareAreas);
                        if (detectedObj != null)
                        {
                            String msg = String.format(
                                "BackVision(%s): targetPose=%s, robotPose=%s",
                                backCamPipeline, detectedObj.targetPose, detectedObj.robotPose);
                            dashboard.putString(DBKEY_PREFIX + "BackCam", msg);
                            dashboard.displayPrintf(lineNum++, msg);
                        }
                        else
                        {
                            lineNum++;
                        }
                    }
                }
            }
        }

        return lineNum;
    }   //updateStatus

}   //class Vision
