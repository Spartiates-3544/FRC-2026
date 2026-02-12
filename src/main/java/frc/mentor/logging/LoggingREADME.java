package frc.mentor.logging;

/**
 * API exposée aux jeunes (logging / AdvantageKit)
 *
 * - Log.value(String key, double v)
 * - Log.value(String key, int v)
 * - Log.value(String key, boolean v)
 * - Log.value(String key, String v)
 *
 * - Log.event(String key, String msg)
 * - Log.warn(String key, String msg)
 * - Log.error(String key, int code, String msg)
 * - Log.clearError(String key)
 *
 * - Log.time(String name, Runnable fn)
 * - {@code <T> T Log.time(String name, java.util.function.Supplier<T> fn)}
 * - AutoCloseable Log.scope(String name)
 *
 * - Log.everyN(int n, Runnable fn)
 * - Log.oncePerSecond(String key, Runnable fn)
 */
public final class LoggingREADME {
  private LoggingREADME() {}
}
