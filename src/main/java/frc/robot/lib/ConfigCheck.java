package frc.robot.lib;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Preferences;

import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

public final class ConfigCheck{

    static public void SaveCheck(String key, TalonFX talon) {
        // code to get config
        var fx_cfg = new TalonFXConfiguration();
        // fetch *all* configs currently applied to the device
        talon.getConfigurator().refresh(fx_cfg);

        String chk = ComputeCheck(fx_cfg.serialize());

        Preferences.setString(key, chk);
    }

    static public void VerifyCheck(String key, TalonFX talon) {
        
    }

    static private String ComputeCheck(String input) {
         try {
            // Create MessageDigest instance for MD5
            MessageDigest md = MessageDigest.getInstance("MD5");

            // Update the digest with the input string bytes
            md.update(input.getBytes());

            // Get the hash's bytes 
            byte[] bytes = md.digest();

            // Convert the bytes to a hexadecimal string
            StringBuilder sb = new StringBuilder();
            for (byte b : bytes) {
                sb.append(String.format("%02x", b));
            }

            return sb.toString();

        } catch (NoSuchAlgorithmException e) {
            return "";
        }
    }
}
