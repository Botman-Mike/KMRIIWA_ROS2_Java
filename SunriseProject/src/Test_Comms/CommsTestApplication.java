package Test_Comms;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.task.ITaskLogger;
import javax.inject.Inject;

public class CommsTestApplication extends RoboticsAPIApplication {
    @Inject
    private ITaskLogger logger;
    
    private volatile boolean isRunning = true;
    private CommsTester commsTester;

    @Override
    public void initialize() {
        logger.info("Initializing Communications Test Application");
        commsTester = new CommsTester(true, logger); // true = running on robot
    }

    @Override
    public void run() {
        try {
            commsTester.startAll();
            
            // Keep application running until shutdown
            while (isRunning) {
                Thread.sleep(1000);
            }
        } catch (Exception e) {
            logger.error("Error in communications test: " + e.getMessage());
        } finally {
            commsTester.stopAll();
        }
    }

    @Override
    public void dispose() {
        isRunning = false;
        if (commsTester != null) {
            commsTester.stopAll();
        }
        super.dispose();
    }
}