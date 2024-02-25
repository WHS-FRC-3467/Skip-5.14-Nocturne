package frc.robot.Util;

import java.util.ArrayList;
import java.util.List;

public class ShooterConfig {
    private List<ShooterPreset> shooterConfigs;

    public ShooterConfig() {
        shooterConfigs = new ArrayList<ShooterPreset>();
    }

    public ShooterConfig(ArrayList<ShooterPreset> pShooterConfigs) {
        this.shooterConfigs = pShooterConfigs;
    }

    public List<ShooterPreset> getShooterConfigs() {
        return shooterConfigs;
    }
}
