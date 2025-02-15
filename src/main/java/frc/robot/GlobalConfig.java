package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.path.PathPlannerPath;

public final class GlobalConfig {
    public static final int version = 2025;

    private static HashMap<String, PathPlannerPath> aimPathDic = new HashMap<String,PathPlannerPath>();
    private static final int[] aimAprilTagIds = new int[] {
        6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22,
    };

    // private static final String[] aimPathNames = new String[] {
    //     "ap10_left",
    //     "ap10_right",
    //     "ap11_left",
    //     "ap11_right",
    //     "ap6_left",
    //     "ap6_right",
    //     "ap7_left",
    //     "ap7_right",
    //     "ap8_left",
    //     "ap8_right",
    //     "ap9_left",
    //     "ap9_right",
    //     "ap_17_left",
    //     "ap_17_right",
    //     "ap_18_left",
    //     "ap_18_right",
    //     "ap_19_left",
    //     "ap_19_right",
    //     "ap_20_left",
    //     "ap_20_right",
    //     "ap_21_left",
    //     "ap_21_right",
    //     "ap_22_left",
    //     "ap_22_right",
    // };

    
    private static String getApPathName(int apId, boolean isLeft) {
        return "ap_" + apId + "_" + (isLeft ? "left" : "right");
    }

    public static boolean init() {
        try {
            for (int i = 0; i < aimAprilTagIds.length; ++i) {
                for (int j = 0; j < 2; ++j) {
                    String pathName = getApPathName(aimAprilTagIds[i], j == 0);
                    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                    System.out.println("=========== " + pathName + " loaded ===============");
                    aimPathDic.put(pathName, path);
                }
                
            }

            return true;
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return false;
    }


    public static PathPlannerPath getAimPath(String name) {
        if (!aimPathDic.containsKey(name)) {
            return null;
        }

        return aimPathDic.get(name);
    }
}
