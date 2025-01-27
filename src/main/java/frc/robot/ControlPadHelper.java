package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.IntegerArrayEntry;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlPadHelper {
    public static class ControlPadInfo {
        public long aprilTagId = -1;
        public long level = 0;
        public long branch = -1;
    }
    private static final String name = "5515ControlPad";
    private static final long[] nullLongArray = new long[0];
    private static ControlPadInfo controlPadInfo = new ControlPadInfo();

    private static NetworkTableInstance ntInst = null;
    private static NetworkTable ntTable = null;
    private static DoubleArrayPublisher robotPosePublisher = null;
    private static IntegerArrayTopic controlPadInfoTopic = null;
    private static IntegerArrayEntry controlPadInfoEntry = null;
    private static NetworkTableInstance getNTInst() {
        if (ntInst != null) {
            return ntInst;
        }

        ntInst = NetworkTableInstance.getDefault();
        return ntInst;
    }
    private static NetworkTable getNTTable() {
        if (ntTable != null) {
            return ntTable;
        }
        ntTable = getNTInst().getTable(name);
        return ntTable;
    }

    private static void Flush() {
        getNTInst().flush();
    }

    // private static NetworkTableEntry getNTTableEntry(String entryName) {
    //     getNTTable().getTo
    //     return getNTTable().getEntry(entryName);
    // }

    private static DoubleArrayPublisher getRobotPosPublisher() {
        if (robotPosePublisher != null) {
            return robotPosePublisher;
        }
        String entryName = "RobotPos";
        DoubleArrayTopic daTopic = getNTInst().getDoubleArrayTopic(entryName);
        robotPosePublisher = daTopic.publish(PubSubOption.keepDuplicates(true));
        return robotPosePublisher;
    }

    public static void publishRobotPos(Pose2d pos) {
        getRobotPosPublisher().set(new double[]{pos.getTranslation().getX(), pos.getTranslation().getY(), pos.getRotation().getDegrees()});
        // getNTTableEntry(entryName).setDoubleArray(new double[]{pos.getTranslation().getX(), pos.getTranslation().getY(), pos.getRotation().getDegrees()});
        Flush();
    }

    private static IntegerArrayTopic getControlPadInfoTopic() {
        if (controlPadInfoTopic != null) {
            return controlPadInfoTopic;
        }
        String entryName = "ControlPadInfo";
        controlPadInfoTopic = getNTTable().getIntegerArrayTopic(entryName);
        return controlPadInfoTopic;
    }

    private static IntegerArrayEntry getControlPadInfoEntry() {
        if (controlPadInfoEntry != null) {
            return controlPadInfoEntry;
        }
        controlPadInfoEntry = getControlPadInfoTopic().getEntry(nullLongArray);
        return controlPadInfoEntry;
    }
    public static void refreshControlPad() {
        long[] datas = getControlPadInfoEntry().get();
        // datas[0] is apriltag id
        // datas[1] is level of branch. 0 is bottom, 1 is 1st level, 2 is 2nd level, 3 is 3rd level
        // datas[2] is the left or right branch. -1 is left, 1 is right, 0 means level is bottom
        if (datas.length == 0) {
            return;
        }
        controlPadInfo.aprilTagId = datas[0];
        controlPadInfo.level = datas[1];
        controlPadInfo.branch = datas[2];

        SmartDashboard.putNumber("ControlPad aprilTagId", controlPadInfo.aprilTagId);
        SmartDashboard.putNumber("ControlPad level", controlPadInfo.level);
        SmartDashboard.putNumber("ControlPad branch", controlPadInfo.branch);
    }

    public static ControlPadInfo getControlInfo() {
        if (controlPadInfo.aprilTagId == -1) {
            return null;
        }
        return controlPadInfo;
    }

}
