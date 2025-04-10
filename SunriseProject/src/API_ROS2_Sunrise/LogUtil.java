package API_ROS2_Sunrise;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import javax.inject.Inject;
import com.kuka.task.ITaskLogger;

public class LogUtil {
    private static ITaskLogger logger;
    // Explicitly specify types for Java 6 compatibility
    private static Map<String, Long> lastLogTimes = new ConcurrentHashMap<String, Long>();
    private static final long THROTTLE_MS = 5000;
    
    @Inject
    public void setLogger(ITaskLogger injectedLogger) {
        logger = injectedLogger;
        // Add initialization log to verify logger is working
        if (logger != null) {
            logger.info("Logger initialized successfully");
        }
    }
    
    // For direct messages without throttling
    public static void logInfo(String message) {
        if (logger != null) {
            logger.info(message);
        } else {
            // Fallback to System.out for debugging logger initialization issues
            System.out.println("WARNING: Logger not initialized - " + message);
        }
    }
    
    // For throttled messages with a key
    public static void logInfo(String key, String message) {
        if (shouldLog(key)) {
            logInfo(message);
        }
    }
    
    public static void logError(String message) {
        if (logger != null) {
            logger.error(message);
        }
    }
    
    public static void logError(String key, String message) {
        if (shouldLog(key)) {
            logError(message);
        }
    }
    
    // Add these new overloads for Exception handling
    public static void logInfo(Exception e) {
        if (logger != null) {
            logger.info(e.toString());
        }
    }

    public static void logInfo(String key, Exception e) {
        if (shouldLog(key)) {
            logInfo(e);
        }
    }

    public static void logError(Exception e) {
        if (logger != null) {
            logger.error(e.toString());
        }
    }

    public static void logError(String key, Exception e) {
        if (shouldLog(key)) {
            logError(e);
        }
    }
    
    private static boolean shouldLog(String key) {
        long now = System.currentTimeMillis();
        Long lastLog = lastLogTimes.get(key);
        if (lastLog == null || now - lastLog > THROTTLE_MS) {
            lastLogTimes.put(key, now);
            return true;
        }
        return false;
    }
    
    public static void resetLogThrottle(String key) {
        lastLogTimes.remove(key);
    }
}