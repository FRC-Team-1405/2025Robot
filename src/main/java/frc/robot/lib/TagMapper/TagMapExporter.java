package frc.robot.lib.TagMapper;

import edu.wpi.first.math.geometry.*;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;

import org.json.simple.JSONObject;

public class TagMapExporter {
    public static void export(Map<Integer, Pose3d> tagMap, String filePath) {
        JSONObject root = new JSONObject();

        for (Map.Entry<Integer, Pose3d> entry : tagMap.entrySet()) {
            JSONObject poseJson = new JSONObject();
            Pose3d pose = entry.getValue();

            poseJson.put("x", pose.getX());
            poseJson.put("y", pose.getY());
            poseJson.put("z", pose.getZ());
            poseJson.put("rotX", pose.getRotation().getX());
            poseJson.put("rotY", pose.getRotation().getY());
            poseJson.put("rotZ", pose.getRotation().getZ());

            root.put("tag_" + entry.getKey(), poseJson);
        }

        try (FileWriter file = new FileWriter(filePath)) {
            file.write(root.toJSONString());
            file.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
