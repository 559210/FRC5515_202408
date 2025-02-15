package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.path.PathPlannerPath;

public final class GlobalConfig {
    public static final int version = 2025;
    public static boolean isTuningMode = false;

    private static HashMap<String, PathPlannerPath> aimPathDic = new HashMap<String,PathPlannerPath>();
    private static final String[] aimPathNames = new String[] {
        "ap17_left",
        "ap17_right",
    };

    public static boolean init() {
        try {
            for (int i = 0; i < aimPathNames.length; ++i) {
                String pathName = aimPathNames[i];
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                System.out.println("=========== " + pathName + " loaded ===============");
                aimPathDic.put(pathName, path);
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
