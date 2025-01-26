package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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


    private static NetworkTable getNTTable() {
        return NetworkTableInstance.getDefault().getTable(name);
    }

    private static void Flush() {
        NetworkTableInstance.getDefault().flush();
    }

    private static NetworkTableEntry getNTTableEntry(String entryName) {
        return getNTTable().getEntry(entryName);
    }

    public static void publishRobotPos(Pose2d pos) {
        String entryName = "RobotPos";
        getNTTableEntry(entryName).setDoubleArray(new double[]{pos.getTranslation().getX(), pos.getTranslation().getY(), pos.getRotation().getDegrees()});
        Flush();
    }

    public static void refreshControlPad() {
        String entryName = "ControlPadInfo";
        long[] datas = getNTTableEntry(entryName).getIntegerArray(nullLongArray);
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
