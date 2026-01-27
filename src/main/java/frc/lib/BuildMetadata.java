package frc.lib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.generated.BuildConstants;

public class BuildMetadata implements Sendable {
    
    @Override 
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Build Metadata");
        builder.addStringProperty("version", ()->{return BuildConstants.VERSION;}, null);
        builder.addStringProperty("Git Date", ()->{return BuildConstants.GIT_DATE;}, null);
        builder.addStringProperty("Git Branch", ()->{return BuildConstants.GIT_BRANCH;}, null);
        builder.addStringProperty("Deploy Date", ()->{return BuildConstants.BUILD_DATE;}, null);
        builder.addBooleanProperty("Uncomitted Changes", () -> {return BuildConstants.DIRTY > 0 ;}, null);
    }
}
